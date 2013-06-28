function error_checking_for_DARE(Sss,eig_ss,Lss,report)
chol(Sss);  %% Sss has to positive definite
if any(eig_ss>=1)
    error('Ali: we have unstable modes');
end
if report==-1 || report==-2
    error('Ali: "Report" is not normal')
end
end