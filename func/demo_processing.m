function [Data,demo_dura] = demo_processing(demos,demoLen,demo_dt,alpha)
demoNum = 1;
demo_dura=1/demo_dt; 
demo_scale =1;
dim=2; 


  totalNum=0;
  for i=1:demoNum        
    for j=1:demoLen
        totalNum=totalNum+1;
        Data(1,totalNum)=(j);
        % Data(2,totalNum)=(j)^2;
        Data(2,totalNum)=exp(alpha*j/demoLen);%修改
        dTmp = spline(1:size(demos{i}.pos,2), demos{i}.pos, linspace(1,size(demos{i}.pos,2),demoLen)); %Resampling
        Data(3:4,totalNum)=dTmp(1:dim,j);  
        if j<demoLen
            Data(5:6,totalNum)=(dTmp(1:dim,j+1)-dTmp(1:dim,j));
        else 
            Data(5:6,totalNum)=Data(5:6,totalNum-1);
        end
    end 
  end

  if demoNum <6
        Data = [Data Data+[0;0;demo_scale ;demo_scale; demo_scale ;demo_scale ] Data+[0;0;-demo_scale ;-demo_scale; -demo_scale ; -demo_scale ]];
  end

end

