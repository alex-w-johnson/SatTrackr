close all, clear all, clc;

portName = 'COM3';
baudRate = 9600;
writeRate = 100; %Hz, rate at which bytes are written to board

% Open and Create Serial Object
serialObj = serial(portName,'BaudRate',baudRate);
fopen(serialObj);
readData = fscanf(serialObj);
writeData = uint8(round(255*rand(10,1)));
dataInString = [''];
for i1 = 1:length(writeData)
%     fprintf(serialObj,'%u',round(255*rand()));
    dataInString = [dataInString,' | ',num2str(writeData(i1))];
%     serialOut{i1} = fread(serialObj,'%f','delimiter','\n');
end
fwrite(serialObj,dataInString);
fclose(serialObj);
delete(serialObj);
clear serialObj