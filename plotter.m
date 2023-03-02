% figure(1)
% subplot(3,1,1)

plot(p_im_REF_sim, 'bo-', 'MarkerSize', 2)
hold on
plot(p_im_sim, 'rs--', 'MarkerSize', 2)

xlabel('Time [s]')
ylabel('Intake pressure [Pa]')
title('Intake pressure VS Intake pressure reference, no feedback')

legend('Intake pressure reference', 'Intake pressure', 'Location', 'southeast')

% grid on
% set(gca,'GridLineStyle','--')