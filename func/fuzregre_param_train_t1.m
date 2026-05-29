function [p1_u1] = fuzregre_param_train_t1(Data_train, U, N_C, Location_X, Location_Y)
[~, jl] = max(U);

slx=size(Location_X,2);
sly=size(Location_Y,2);
p1_u1=zeros(1+slx,sly,N_C);



for i=1:N_C    
 idx = (jl==i);
 zc = Data_train(:,idx);

 Wi = U(i,idx);
 Wii = Wi'*ones(1,1+slx);
 X1=[ones(size(zc,2),1),zc(Location_X,:)']; 
    
 Y1=zc(Location_Y,:)';
 p1_u1(:,:,i)=(X1'*(Wii.*X1))^(-1)*X1'*(Wi'.*Y1);


end

end



