function uTransVec = uTRans_hc_12112018()

 load('refpath1218.mat')
 load('controlu.mat')
load('mpc_df.mat')
%write values for each point in time
for i = 1:length(ref.Xe)
    %R
    %{
    uTransVec(1,i) =0;
    uTransVec(2,i) =0.12;
    uTransVec(3,i) =0.10;
    uTransVec(4,i) =0;
    uTransVec(5,i) =0;
    uTransVec(6,i) =0;
    %}
    load('refpath1218.mat')
    load('vyy.mat')
    load('ppsidot.mat')
    %Xn
    %{
    uTransVec(7,i) = ref.Xe(i);
    uTransVec(8,i) = ref.Ye(i);
    uTransVec(9,i) = ref.Psi(i);
    uTransVec(10,i) = ref.Xdot(i);
    uTransVec(11,i) = vyy(i);
    uTransVec(12,i) = ppsidot(i);
%%
    lf=2.3; %m
    lr=2.25; %m
    W=atan((lf+lr)*1.3539/25);
    %W
    uTransVec(13,i) = W;
    %}
    
    uTransVec(1,i) =ScopeDatau.signals.values(i);
    uTransVec(2,i) = ref.Xe(i);
    uTransVec(3,i) = ref.Ye(i);
    uTransVec(4,i) = ref.Psi(i);
    uTransVec(5,i) = ref.Xdot(i);
    uTransVec(6,i) = vyy(i);
    uTransVec(7,i) = ppsidot(i);
end



