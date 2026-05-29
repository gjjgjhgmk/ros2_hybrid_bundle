function [C,pInvCov, p1_u] = fuzzymodellingCandGK(Data,demoLen,N_C,maxIter_fcm,maxIter_gk)

[C,AFPCD,pInvCov,cov,U] = fuzzy_clustering(Data,demoLen,N_C,maxIter_fcm,maxIter_gk);



[p1_u] = fuzregre_param_train_t1(Data, U, N_C, [1 2], [3]);

end

