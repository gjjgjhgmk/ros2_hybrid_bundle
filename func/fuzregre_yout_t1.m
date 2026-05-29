function [y1, U] = fuzregre_yout_t1(Data_test,demo_dura,alpha,N_C, C, pInvCov, p1_u, Location_X, Location_Y)

m = 2;
% Data_test = [demo_dura*Data_test;(demo_dura*Data_test).^2];
Data_test = [demo_dura*Data_test;exp(alpha*Data_test/Data_test(end))];%修改
N_Data = size(Data_test,2);
D_y=size(Location_Y,2);
MD2 = zeros(N_C,N_Data);
for i = 1:N_C
    dist = Data_test([Location_X],:) - C([Location_X],i);
    pInvCov_i = pInvCov(:,:,i);
    pInvCov_i = pInvCov_i([Location_X],[Location_X]);
    MD2(i,:) = sum(dist.*(pInvCov_i*dist), 1);
end

temp = 1./MD2.^(1/(m-1));

U = (temp./(ones(N_C,1)*sum(temp)));


y1=zeros(D_y,N_Data);

for j=1:N_C
         y1=([ones(N_Data,1) Data_test(Location_X,:)']*p1_u(:,:,j))'.*U(j,:)+y1;
      
end 

end

