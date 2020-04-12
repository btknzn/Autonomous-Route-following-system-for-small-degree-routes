%%Proje of Batu Kaan Özen.
%Projin 1. k?sm? 0.001 derece aç?dan dolay? 10sn de araban?n yanl??
%gitti?i ( Yani sistemin y ekseninde kayd??? miktar)
%mesafe hesaplanm??t?r.
clc
clear
Am = [ 1 1 1/2;0 1 1;0 0 1];
Bm = [0 ; 0; 1/80];
Cm = [1 0 0];
Dm = 1;
system=ss(Am,Bm,Cm,Dm);
u=ones(1,10000)*0.0001;
t = linspace(0,10,10000);
systemout=sin(lsim(system,u,t))
figure()
plot(t,systemout)
title('system out for +0.0001 degree. You can see for straight line how manymeter is straight going system mistake going to have in 10 Sec ')
%System simulink ç?kt?s?
model = sim('projemodel1.slx')
value = model.systemout
plot(value)
title('System out')

% Projenin 2. k?sm?nda sistemdeki sistemimizi do?rusalla?t?rma i?lemi
% yap?yoru bunun için sin(sistemout) olan do?rusal olmayan ifademizi
% küçük aç?larda do?rusalla?t?r?yoruz bu do?rusalla?t?rma i?leminden dolay?
%sistem direk sistemout ?eklinde ç?k?? veriyor.
%do?rusalla?t?r?lm?? sistem Do?rusalla?t?r?lm?i? sistemimizin
%tek fark? sistemizden sin de?erinin gitmesi olmu?tur bundan dolay?
% A B C ve D de?erlerimiz tamamen ayn? kalm??t?r
% Zaten ilk k?s?mda sistem ziyan edilirken sin(lsim(system,..)) ?eklinde
% yap?lm??t?r ondan dolay? art?k sistem ç?k???m?z lsim(system,...))
% olucakt?r ilk olarak sin(lsim(system,...)) do?rusalla?t?rd???m?z
% sistemden Am Bm Cm Dm say?lar?n? çekelim
Aml = system.A;
Bml = system.B;
Cml = system.C;
Dml = system.D;
%systeml bizim linear sistemimiz sin(lsim(sistem)) ise bizim linear olmayan
%sistemimiz
systeml = ss(Aml,Bml,Cml,Dml);

sysTum = ss(Aml,[Bml Bml], Cml, [Dml 1]);
u = u +0.000001*randn(size(u));
y2 = systemout +0.1* randn(size(u))';
u=u';
% Sistemimiz matlab ortam?nda farkl? parametreler için a?a??daki gibi
% denenmi?tir.
KalmanLin1 = kalman(sysTum,1,1);
KalmanLin2 = kalman(sysTum,1,2);
KalmanLin3 = kalman(sysTum,2,1);
lsim(KalmanLin1,[u y2],t)
title('Q=1 R=1 iken')
lsim(KalmanLin2,[u y2],t)
title('Q=1 R=2 iken')
lsim(KalmanLin3,[u y2],t)
title('Q=2 R=1 iken')

% Sistemimiz farkl? kalman parametreli (1-1) (1-2) (2-1) ile  a?a??daki
% gibi deniyoruz

model = sim('projemodel2.slx');
Kalman21 = model.kalman21;
Kalman11 = model.kalman11;
Kalman12 = model.kalman12;
sistemout = model.Sistemout;
figure()
subplot(4,1,1)
plot(Kalman21)
title(' Q = 2 R=1')
subplot(4,1,2)
plot(Kalman11)
title(' Q = 1 R=1')
subplot(4,1,3)
plot(Kalman12)
title(' Q = 1 R=2')
subplot(4,1,4)
plot(sistemout)
title('Orjinal sistem')

% Projenin 3. k?sm?
