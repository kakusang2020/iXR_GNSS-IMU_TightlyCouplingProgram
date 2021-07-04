function Compar2filevel(inputArg1,inputArg2)
result = [];inputArg2(:,1)=inputArg2(:,1);
for i = 1:length(inputArg2)
    index = find(inputArg1(:,1) == inputArg2(i,1));
    if ~isnan(index)
        try
    result=[result;inputArg1(index,1),inputArg1(index,5:7),inputArg2(i,9),inputArg2(i,8),inputArg2(i,10)];
        catch
            disp('???');
        end
    else
    end
end

disp('max error is :');
fprintf('%6.4f,%6.4f,%6.4f\n',max(abs(result(:,2)-result(:,5))),max(abs(result(:,3)-result(:,6))),max(abs(result(:,4)-result(:,7))));
disp('mean error is :');
fprintf('%6.4f,%6.4f,%6.4f\n',mean(result(:,2)-result(:,5)),mean(result(:,3)-result(:,6)),mean(result(:,4)-result(:,7)));
figure;
subplot(3,1,1);
plot(result(:,1),result(:,2)-result(:,5),'LineWidth',4);
title('Error in E-W(m/s)');
set(gca,'FontSize',20);
grid on;
subplot(3,1,2);
plot(result(:,1),result(:,3)-result(:,6),'LineWidth',4);
set(gca,'FontSize',20);
title('Error in N-S(m/s)');
grid on;
subplot(3,1,3);
plot(result(:,1),result(:,4)-result(:,7),'LineWidth',4);
set(gca,'FontSize',20);
title('Error in U-D(m/s)');
grid on;
end