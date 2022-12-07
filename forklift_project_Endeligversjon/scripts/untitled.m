%%
%Robot arm
for d = 1
    
L1=0.50;
L2=1.00;
L3=1.00;
L4=1.00;
L5=0.50;
L6=0.50;

%DH paramters to make kinematic model
j1 = Revolute('d', 0, 'a', L1, 'alpha', pi/2, 'offset', 0);
j2 = Revolute('d', 0, 'a', L2, 'alpha', 0, 'offset', 0);
j3 = Revolute('d', 0, 'a', L3, 'alpha', 0, 'offset', 0);
j4 = Revolute('d', 0, 'a', L4, 'alpha', 0, 'offset', 0);
j5 = Revolute('d', 0, 'a', L5, 'alpha', -pi/2, 'offset', 0);
j6 = Revolute('d', 0, 'a', L6, 'alpha', 0, 'offset', 0);

my_robot =  SerialLink([j1 j2 j3 j4 j5 j6],'name', 'my robot');


%Forward kinematics toolbox
T0 = my_robot.fkine([0, 0, 0, 0, 0, 0]);



%inverse kinematics
palle = transl(3, 0, -1) * trotx(0);

palleLoft = transl(3, 0, -0.9) * trotx(0);

taVekk = transl(2.5, 1, 0) *trotx(0);

taPaPlass = transl(3, 2, 1.1) * trotx(0);

taPaPlassSettNed = transl(3, 2, 1) * trotx(0);

palle = transl(3.5, 0, -1) * rpy2tr(0,0,0,'deg');

palleLoft = transl(3.5, 0, -0.9) * rpy2tr(0,0,0, 'deg');

taVekk = transl(2, 0, 0) *rpy2tr(0,0,0, 'deg');

taPaPlass = transl(3, 2, 1.1) * rpy2tr(0,0,0, 'deg');

taPaPlassSettNed = transl(3, 2, 1) * rpy2tr(0,0,0, 'deg');

%hold on
%trplot(palle)

%hold on 
%trplot(taPaPlassSettNed)

v1 = ctraj(palle, palleLoft, 50);

v2 = ctraj(palleLoft, taVekk, 50);

v3 = ctraj(taVekk, taPaPlass, 50);

v4 = ctraj(taPaPlass, taPaPlassSettNed, 50);

rV1 = my_robot.ikcon(v1);

rV2 = my_robot.ikcon(v2);

rV3 = my_robot.ikcon(v3);

rV4 = my_robot.ikcon(v4);

%my_robot.plot(rV1)

%hold on

%my_robot.plot(rV2)

%hold on

%my_robot.plot(rV3)

%hold on

%my_robot.plot(rV4)

%figure
%qplot(v2)

%figure 
%qplot(v3)
end


%%
%Mobile platform
for d = 1
    out = sim('sl_pursuit_negativ_gammaogV_EndeligTune');
    y = out.find('y');
    t = out.find('t');
end