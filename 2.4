function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
%{ 
inputs:
Fc - 3x1 control force vec in body frame
Fc = [Fx,Fy,Fz]^T, only Fz used
Gc - 3x1 control moment vec in body frame
Gc = [Lc,Mc,Nc]^T, roll pitch yaw moments
d = distance from cg to each rotor
km - control moment (yaw) coefficient
%}

%outputs 4x1 column vector [f1,f2,f3,f4]^T

%{
[Zc]   [    -1        -1         -1        -1    ] [f1]
[Lc] - [-d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2)] [f2]
[Mc] - [d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2)] [f3]
[Nc]   [  km         -km        km         -km   ] [f4]
%}





Zc = Fc(3);
Lc = Gc(1);
Mc = Gc(2);
Nc = Gc(3);

a = d/sqrt(2);

A = [-1, -1, -1, -1;
     -a, -a,  a,  a;
      a, -a, -a,  a;
     km, -km, km,-km];

b = [Zc;Lc;Mc;Nc];


motor_forces = A \ b;

end
