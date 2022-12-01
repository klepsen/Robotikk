close all
import ETS3.*

L1 = 0.5;
L2 = 1.0;
L3 = 1.0;
L4 = 1.0;
L5 = 0.5;
L6 = 0.5;

robot_normal = Tz(L1) * Rz('q1') * Tz(L2) * Ry('q2') * Tz(L3) * Ry('q2') * Tz(L4) * Ry('q3') * Tz(L4) * Ry('q4') * Tz(L5) * Ry('q5') * Tz(L6) * Rz('q6');
robot_normal.plot([0,0,0,0,0,0])
