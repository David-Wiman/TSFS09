figure(2)
subplot(2,2,1)
plot(p_im_REF_sim,'bo-', 'MarkerSize', 2)
hold on
plot(p_im_sim, 'rs--', 'MarkerSize', 2)
xlabel('Time [s]')
ylabel('Intake pressure [Pa]')
title('Intake pressure VS reference')
legend('Intake pressure reference', 'Intake pressure', 'Location', 'southwest')

subplot(2,2,2)
plot(p_ic_REF_sim,'bo-', 'MarkerSize', 2)
hold on
plot(p_ic_sim, 'rs--', 'MarkerSize', 2)
xlabel('Time [s]')
ylabel('Boost pressure [Pa]')
title('Boost pressure VS reference')
legend('Boost pressure reference', 'Boost pressure', 'Location', 'southwest')

subplot(2,2,3)
plot(alpha_REF_sim, 'bo-', 'MarkerSize', 2)
xlabel('Time [s]')
ylabel('Alpha reference [-]')
title('Alpha reference')

subplot(2,2,4)
plot(wg_REF_sim, 'bo-', 'MarkerSize', 2)
xlabel('Time [s]')
ylabel('Wastegate reference [-]')
title('Wastegate reference')
