subplot 511

time = scope_MainSensors.time;
Comm = scope_MainSensors.signals(1).values;
Force = scope_MainSensors.signals(2).values;
Roll = scope_MainSensors.signals(3).values;
Angle = scope_MainSensors.signals(4).values;
Mode = scope_system_mode.signals(2).values;

subplot 411
plot(time, Comm,"Color",'r','LineWidth',1)
legend('Ail_{com}')
axis([0 20 -0.2 0.2])
subplot 412
plot(time,Roll,"LineWidth",1)
legend('\phi','\phi^\prime','\phi^{\prime\prime}')
axis([0 20 -0.8 1.8])
subplot 413
plot(time,Angle,"Color",'r',LineWidth=1)
legend('\lambda','\lambda^\prime')
% subplot 414
% plot(time,Force,'Color','r','LineWidth',1)
% legend('Hydrolic Force in piston 1')
% axis([0 20 0 50000])
subplot 414
plot(time,Mode,'Color','r','LineWidth',1)
legend('Mode of system')
axis([0 20 0 11])
xlabel('time[s]')