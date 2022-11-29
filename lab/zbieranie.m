delete(instrfindall); % zamkniecie wszystkich polaczen szeregowych
clear all;
close all;
% s = serial('COM6'); % COM9 to jest port utworzony przez mikrokontroler
% set(s,'BaudRate',115200);
% set(s,'StopBits',1);
% set(s,'Parity','none');
% set(s,'DataBits',8);
% set(s,'Timeout',1);
% set(s,'InputBufferSize',1000);
% set(s,'Terminator',13);
s = serialport('COM6', 115200, 'Parity', 'None');
s.configureTerminator('LF');
fopen(s); % otwarcie kanalu komunikacyjnego
Tp = 0.1; % czas z jakim probkuje regulator
y = []; % wektor wyjsc obiektu
u = []; % wektor wejsc (sterowan) obiektu
% Y_zad(1:1:10) = 1000;
while length(y)~=600 % zbieramy 100 pomiarow
txt = s.readline(); % odczytanie z portu szeregowego
% txt powinien zawiera´c Y=%4d;U=%4d;
% czyli np. Y=1234;U=3232;
eval(char(txt')); % wykonajmy to co otrzymalismy
y=[y;Y]; % powiekszamy wektor y o element Y
u=[u;U]; % powiekszamy wektor u o element U

end
clear s;

figure(1); grid on; hold on;plot((0:(length(y)-1))*Tp,y);title('Wyjście');% wyswietlamy y w czasie
figure(2); grid on; plot((0:(length(u)-1))*Tp,u); title('Sterowanie'); % wyswietlamy u w czasie

save('Y_test_320', "y");
save('U_test_320', "u");
