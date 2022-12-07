close all
import ETS3.*

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


%%
for d = 1
rosinit
   
[pub_q1,msg_q1] = rospublisher('/mobile_manipulator_forklift/arm_base_controller/command','std_msgs/Float64');
[pub_q2,msg_q2] = rospublisher('/mobile_manipulator_forklift/link_1_controller/command','std_msgs/Float64');
[pub_q3,msg_q3] = rospublisher('/mobile_manipulator_forklift/link_2_controller/command','std_msgs/Float64');
[pub_q4,msg_q4] = rospublisher('/mobile_manipulator_forklift/link_3_controller/command','std_msgs/Float64');
[pub_q5,msg_q5] = rospublisher('/mobile_manipulator_forklift/link_4_controller/command','std_msgs/Float64');
[pub_q6,msg_q6] = rospublisher('/mobile_manipulator_forklift/link_5_controller/command','std_msgs/Float64');
[pub_vel,msg_vel] = rospublisher('/mobile_manipulator_forklift/ackermann_steering_controller/cmd_vel','geometry_msgs/Twist');

msg_vel.Linear.X = 0;
msg_vel.Angular.Z = 0;

qstart = [2.80611740951817e-06,0.414390417153111,-3.14159265358979,2.72493777613769,0.00254994746307481,1.83014137815377e-06];

        msg_q1.Data = qstart(1);
        msg_q2.Data = -qstart(2);
        msg_q3.Data = -qstart(3);
        msg_q4.Data = -qstart(4);
        msg_q5.Data = -qstart(5);
        msg_q6.Data = -qstart(6);
        
        
        send(pub_q1,msg_q1)
        send(pub_q2,msg_q2)
        send(pub_q3,msg_q3)
        send(pub_q4,msg_q4)
        send(pub_q5,msg_q5)
        send(pub_q6,msg_q6)

%starting count from b because matab go brrrrrrrr
teller = 1;
b = 0;
c = 0;
v = 0;
e = 0;
g = 0;
rate = robotics.Rate(60);
while rate.TotalElapsedTime < 95
    ratespeed = robotics.Rate(10);
    for e = 1:473
        if g < 473
            msg_vel.Linear.X = y(1+g,6);
            msg_vel.Angular.Z = y(1+g,5)/t(1+g);
        
     
            send(pub_vel,msg_vel)
            g = g + 1;
            waitfor(ratespeed);
        end
    end
    
     msg_vel.Linear.X = 0;
     msg_vel.Angular.Z = 0;
     send(pub_vel,msg_vel)
    
    rate1 = robotics.Rate(10);
    if teller == 1
      for i = 1:51
          if b < 50
        msg_q1.Data = rV1(b+1,1);
        msg_q2.Data = -rV1(b+1,2);
        msg_q3.Data = -rV1(b+1,3);
        msg_q4.Data = -rV1(b+1,4);
        msg_q5.Data = -rV1(b+1,5);
        msg_q6.Data = -rV1(b+1,6);
        
        
        send(pub_q1,msg_q1)
        send(pub_q2,msg_q2)
        send(pub_q3,msg_q3)
        send(pub_q4,msg_q4)
        send(pub_q5,msg_q5)
        send(pub_q6,msg_q6)
        b = b + 1;
        
          else 
          end
        waitfor(rate1);
      end
      teller = 2;
    end

     rate2 = robotics.Rate(10);
     if teller == 2
      for j = 1:50
          if c < 50
        msg_q1.Data = rV2(c+1,1);
        msg_q2.Data = -rV2(c+1,2);
        msg_q3.Data = -rV2(c+1,3);
        msg_q4.Data = -rV2(c+1,4);
        msg_q5.Data = -rV2(c+1,5);
        msg_q6.Data = -rV2(c+1,6);
        
        
        send(pub_q1,msg_q1)
        send(pub_q2,msg_q2)
        send(pub_q3,msg_q3)
        send(pub_q4,msg_q4)
        send(pub_q5,msg_q5)
        send(pub_q6,msg_q6)
        c = c + 1;
          else 
          end
        waitfor(rate2);

      end
      teller = 3;
    end
        
    ratespeed = robotics.Rate(10);
    for e = 1:472
        if 473 < g || g < 945
      msg_vel.Linear.X = y(1+g,6);
      msg_vel.Angular.Z = y(1+g,5)/t(1+g);
        
     
      send(pub_vel,msg_vel)
      g = g + 1;
        end
      waitfor(ratespeed);
    end
    
     msg_vel.Linear.X = 0;
     msg_vel.Angular.Z = 0;
     send(pub_vel,msg_vel)
      waitfor(rate);
      
          rate3 = robotics.Rate(10);
    if teller == 3
     for j = 1:50
          if v < 50
        msg_q1.Data = rV3(v+1,1);
        msg_q2.Data = -rV3(v+1,2);
        msg_q3.Data = -rV3(v+1,3);
        msg_q4.Data = -rV3(v+1,4);
        msg_q5.Data = -rV3(v+1,5);
        msg_q6.Data = -rV3(v+1,6);
        
        
        send(pub_q1,msg_q1)
        send(pub_q2,msg_q2)
        send(pub_q3,msg_q3)
        send(pub_q4,msg_q4)
        send(pub_q5,msg_q5)
        send(pub_q6,msg_q6)
        v = v + 1;
          else 
          end
        waitfor(rate3);

      end
     teller = 4;
    end
    
        rate4 = robotics.Rate(10);
    if teller == 4
     for r = 1:50
          if e < 50
        msg_q1.Data = rV4(e+1,1);
        msg_q2.Data = -rV4(e+1,2);
        msg_q3.Data = -rV4(e+1,3);
        msg_q4.Data = -rV4(e+1,4);
        msg_q5.Data = -rV4(e+1,5);
        msg_q6.Data = -rV4(e+1,6);
        
        
        send(pub_q1,msg_q1)
        send(pub_q2,msg_q2)
        send(pub_q3,msg_q3)
        send(pub_q4,msg_q4)
        send(pub_q5,msg_q5)
        send(pub_q6,msg_q6)
        e = e + 1;
          else 
          end
        waitfor(rate4);

      end
     teller = 0;
    end
end
end



rosshutdown