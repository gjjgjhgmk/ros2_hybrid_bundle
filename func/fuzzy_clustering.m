
function [C,AFPCD,nIn,cov,U] = fuzzy_clustering(P,len,N_C,maxIter_fcm,maxIter_gk)

[D_P,N_P] = size(P);
onesDPx1 = ones(D_P,1);
ones1xNP = ones(1,N_P);
Ps = P(:,1:len);
sec_2 = floor(0.5*len/(N_C+1));
sec = floor((len-2*sec_2)/(N_C-1));
C = Ps(:,sec_2:sec:end); 
C = C(:,1:N_C);
onesNCx1 = ones(N_C,1);

d2 = @(a,b) sum(bsxfun(@minus,permute(a,[3 2 1]),permute(b,[2 3 1])).^2,3);

dist2 = d2(P,C);
idxs = (dist2 == 0);
dist2(idxs) = 10^(-6);
tmp = 1./dist2;
U = tmp./(onesNCx1*sum(tmp,1));
U2 = U.^2;

for kIter = 1:maxIter_fcm
    rowSumU2 = sum(U2,2);
    C = P*U2'./(onesDPx1*rowSumU2');
    dist2 = d2(P,C);
    tmp = 1./dist2;
    U = tmp./(onesNCx1*sum(tmp,1));
    U2 = U.^2;
end

nIn = zeros(D_P,D_P,N_C);
cov = nIn;

if maxIter_gk <= 0
    eyes = eye(D_P);
    for i = 1:N_C
        nIn(:,:,i) = eyes;
    end
    AFPCD = sum( 1./sum(1./dist2,1) )/N_P;
elseif maxIter_gk == 1
    MD2 = dist2;
    a = 1/D_P;
    rowSumU2 = sum(U2,2);
    C = P*U2'./(onesDPx1*rowSumU2');
    for i = 1:N_C
        dist = P - C(:,i)*ones1xNP;
        U2D = (onesDPx1*U2(i,:)).*dist;
        cov_i = dist*U2D'/rowSumU2(i);
        nIn_i = (det(cov_i)^a)*cov_i^(-1);
        MD2(i,:) = sum(dist.*(nIn_i*dist),1);
        nIn(:,:,i) = nIn_i;
        cov(:,:,i) = cov_i.^0.5;
    end
    AFPCD = sum( 1./sum(1./MD2,1) )/N_P;
else
    MD2 = dist2;
    a = 1/D_P;
    for kIter = 1:maxIter_gk-1
        rowSumU2 = sum(U2,2);
        C = P*U2'./(onesDPx1*rowSumU2');
        for i = 1:N_C
            dist = P - C(:,i)*ones1xNP;
            U2D = (onesDPx1*U2(i,:)).*dist;
            cov_i = dist*U2D'/rowSumU2(i);
            nIn_i = (det(cov_i)^a)*cov_i^(-1);
            MD2(i,:) = sum(dist.*(nIn_i*dist),1);
        end
        tmp = 1./MD2;
        U = tmp./(onesNCx1*sum(tmp,1));
        U2 = U.^2;
    end
    %%%%%% kIter = maxIter
    rowSumU2 = sum(U2,2);
    C = P*U2'./(onesDPx1*rowSumU2');
    for i = 1:N_C
        dist = P - C(:,i)*ones1xNP;
        U2D = (onesDPx1*U2(i,:)).*dist;
        cov_i = dist*U2D'/rowSumU2(i);
        nIn_i = (det(cov_i)^a)*cov_i^(-1);
        MD2(i,:) = sum(dist.*(nIn_i*dist),1);
        nIn(:,:,i) = nIn_i;
        cov(:,:,i) = cov_i.^0.5;
    end
    AFPCD = sum( 1./sum(1./MD2,1) )/N_P;
end

end

