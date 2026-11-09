function tests = test_published_arm_data
%TEST_PUBLISHED_ARM_DATA  The arm's parameters against the paper they came from.
%
%   Gaz et al. (IEEE RA-L 2019), supplementary Table III, lists the dynamic
%   coefficients pi(p-hat) computed from the retrieved parameters. Rebuilding
%   them from grasp.robot.pandaPublishedData checks every transcribed mass, centre of
%   mass and inertia, and fixes the convention: J in the table is about the
%   link frame, and the published I is about the centre of mass.
tests = functiontests(localfunctions);
end

function setupOnce(tc)
R = grasp.robot.pandaPublishedData();
tc.TestData.R = R;
for i = 7:-1:1
    b = R.link(i + 1);
    m(i) = b.mass;  c(i, :) = b.com;
    J(:, :, i) = b.I + b.mass * (dot(b.com, b.com) * eye(3) - b.com.' * b.com);
end
tc.TestData.m = m;  tc.TestData.c = c;  tc.TestData.J = J;
end

function testTableIIICoefficients(tc)
m = tc.TestData.m;  c = tc.TestData.c;  J = tc.TestData.J;
mc = @(i, a) m(i) * c(i, a);
x = 1;  y = 2;  z = 3;
% [ computed value, Table III column pi(p-hat) ]
% Two printed entries are left as follows. Row 2 is printed with
% 0.1067(m3 + ... + m7), which counts m3 twice; 0.1067(m4 + ... + m7) gives
% the tabulated number, and matches the sixth row's form. The coefficient
% c5y m5 - c6z m6 is omitted: Table III gives 0.0824, while Table VIII's c5y,
% m5, c6z, m6 give 0.0679, so the paper's two tables disagree on that one.
T = [ J(2,2,2) + J(3,3,1),                                            0.0373
      J(1,1,2) - J(2,2,2) + J(2,2,3) + 0.0998*m(3) + 0.632*mc(3,z) + 0.1067*sum(m(4:7)),  0.9896
      J(1,2,2),                                                      -0.0040
      J(1,3,2),                                                       0.0103
      J(2,3,2),                                                       0.0008
      J(2,2,3) + J(3,3,2) + 0.09985*m(3) + 0.632*mc(3,z) + 0.1067*sum(m(4:7)),            1.0358
      J(1,1,3) - J(2,2,3) + J(2,2,4) - 0.0068*m(4),                   0.0116
      J(1,3,3),                                                      -0.0055
      J(2,3,3),                                                      -0.0044
      J(2,2,4) + J(3,3,3) + 0.0068*m(4) + 0.0136*sum(m(5:7)),         0.1245
      J(1,1,4) - J(2,2,4) + J(2,2,5) + 0.768*mc(5,z) + 0.1406*sum(m(5:7)),                0.5408
      J(1,2,4) + 0.0317*sum(m(5:7)) + 0.0825*mc(5,z),                 0.1388
      J(2,2,5) + J(3,3,4) + 0.1543*sum(m(5:7)) + 0.768*mc(5,z),       0.6325
      J(2,2,6) + J(3,3,5) + 0.0077*m(7),                              0.0276
      J(2,2,7) + J(3,3,6) + 0.0077*m(7),                              0.0304
      J(1,2,6) + 0.088*mc(7,z),                                       0.0055
      J(3,3,7),                                                       0.0049
      mc(2,x),                                                       -0.0020
      mc(2,y) - 0.316*sum(m(3:7)) - mc(3,z),                         -3.1043
      0.0825*sum(m(4:7)) + mc(3,x),                                   0.6842
      mc(3,y) - mc(4,z),                                              0.0282
      mc(4,x) - 0.0825*sum(m(5:7)),                                  -0.4901
      0.384*sum(m(5:7)) + mc(4,y) + mc(5,z),                          1.7207
      mc(5,x),                                                       -0.0147
      0.088*m(7) + mc(6,x),                                           0.1650
      mc(6,y) - mc(7,z),                                             -0.0688
      mc(7,x),                                                        0.0077
      mc(7,y),                                                       -0.0031];
tc.verifyEqual(T(:, 1), T(:, 2), 'AbsTol', 1.5e-3);
end

function testMovingMassIsInThePapersRange(tc)
% The retrieval constrained the total to 16-20 kg.
tc.verifyGreaterThanOrEqual(sum(tc.TestData.m), 16);
tc.verifyLessThanOrEqual(sum(tc.TestData.m), 20);
end
