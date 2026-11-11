"""Interchangeable demo HSV and pretrained zero-shot language grounders."""
from dataclasses import dataclass
import re
import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    bbox: tuple
    score: float
    label: str


def target_phrase(instruction):
    """Conservative normalization, not a general natural-language parser."""
    phrase = re.sub(r'^\s*(please\s+)?(pick\s+up|grasp|grab|select|find|locate|the)\s+',
                    '', instruction.strip(), flags=re.I)
    phrase = re.sub(r'^\s*(the|a|an)\s+', '', phrase, flags=re.I)
    return phrase.strip(' .!?')


class ColorGrounder:
    """Deterministic tabletop baseline for red/green/blue/yellow commands.

    This is NOT open-vocabulary language grounding. Use ZeroShotGrounder for that.
    """
    HSV_RANGES = {
        'red': [((0, 90, 55), (12, 255, 255)), ((170, 90, 55), (179, 255, 255))],
        'green': [((35, 60, 40), (85, 255, 255))],
        'blue': [((92, 60, 40), (135, 255, 255))],
        'yellow': [((18, 75, 65), (36, 255, 255))],
    }

    def __init__(self, min_area=150):
        self.min_area = min_area

    def detect(self, bgr, instruction):
        phrase = target_phrase(instruction).lower()
        color = next((name for name in self.HSV_RANGES if
                      re.search(r'\b'+name+r'\b', phrase)), None)
        if color is None: return None
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in self.HSV_RANGES[color]:
            mask |= cv2.inRange(hsv, np.array(low), np.array(high))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        if not valid: return None
        best = max(valid, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(best)
        # Score is mask solidity, NOT a calibrated semantic confidence.
        score = min(1.0, cv2.contourArea(best)/(w*h))
        return Detection((x,y,x+w,y+h), float(score), phrase)


class ZeroShotGrounder:
    """Optional Hugging Face zero-shot pipeline, lazily loaded on first use.

    Requires: pip install 'transformers[torch]' pillow.
    The model weights may download from Hugging Face the first time.
    """
    def __init__(self, model='google/owlvit-base-patch32', threshold=0.2):
        self.model = model
        self.threshold = threshold
        self.pipeline = None

    def detect(self, bgr, instruction):
        phrase = target_phrase(instruction)
        if not phrase: return None
        if self.pipeline is None:
            from transformers import pipeline
            self.pipeline = pipeline('zero-shot-object-detection', model=self.model, device=-1)
        from PIL import Image
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        preds = self.pipeline(Image.fromarray(rgb), candidate_labels=[phrase],
                              threshold=self.threshold)
        if not preds: return None
        best = max(preds, key=lambda item: float(item['score']))
        box = best['box']
        # Transformer versions use either xmin/xmax or x_min/x_max.
        def field(a, b): return int(box[a] if a in box else box[b])
        bbox = (field('xmin','x_min'), field('ymin','y_min'),
                field('xmax','x_max'), field('ymax','y_max'))
        return Detection(bbox, float(best['score']), str(best['label']))
