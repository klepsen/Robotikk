      %rV3
      rate4 = robotics.Rate(0.1);
      for k = 51
          if v < 50
        msg_q1.Data = rV3(v+1,1);
        msg_q2.Data = rV3(v+1,2);
        msg_q3.Data = rV3(v+1,3);
        msg_q4.Data = rV3(v+1,4);
        msg_q5.Data = rV3(v+1,5);
        msg_q6.Data = rV3(v+1,6);
        
        
        send(pub_q1,msg_q1)
        send(pub_q2,msg_q2)
        send(pub_q3,msg_q3)
        send(pub_q4,msg_q4)
        send(pub_q5,msg_q5)
        send(pub_q6,msg_q6)
        v = v + 1;
        waitfor(rate4);
          else 
          end

      end
      
            %rV2
      rate3 = robotics.Rate(0.1);
      for k = 51
          if c < 50
        msg_q1.Data = rV2(c+1,1);
        msg_q2.Data = rV2(c+1,2);
        msg_q3.Data = rV2(c+1,3);
        msg_q4.Data = rV2(c+1,4);
        msg_q5.Data = rV2(c+1,5);
        msg_q6.Data = rV2(c+1,6);
        
        
        send(pub_q1,msg_q1)
        send(pub_q2,msg_q2)
        send(pub_q3,msg_q3)
        send(pub_q4,msg_q4)
        send(pub_q5,msg_q5)
        send(pub_q6,msg_q6)
        c = c + 1;
        waitfor(rate3);
          else 
          end

      end
      