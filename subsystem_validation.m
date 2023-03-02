plot(m_dot_c_sim, 'b')
hold on
plot(m_dot_c, '--r')
xlabel('Time [s]')
ylabel('Vehicle Speed [km/h]')
title('Vehicle Speed VS Drive Cycle Speed, smaller vehicle and engine')
legend('Vehicle Speed', 'Drive Cycle Speed')