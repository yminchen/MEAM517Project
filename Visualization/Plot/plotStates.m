
% plot states



hold on
if plot_flag(1)
    plot(T,P(:,1),'b','DisplayName','x (m)') 
end
if plot_flag(2)
    plot(T,P(:,2),'r','DisplayName','y (m)')
end
if plot_flag(3)
    plot(T,P(:,3),'Color',[0 0.5 0],'DisplayName','phi (rad)')
end
if plot_flag(4)
    plot(T,P(:,4),'k','DisplayName','alpha_R (rad)')
end
if plot_flag(5)
    plot(T,P(:,5),'m','DisplayName','beta_R (rad)')
end
if plot_flag(6)
    plot(T,P(:,6),'k','DisplayName','alpha_L (rad)')
end
if plot_flag(7)
    plot(T,P(:,7),'m','DisplayName','beta_L (rad)')
end
if plot_flag(6+2)
    plot(T,P(:,6+2),'b--','DisplayName','xDot (m/s)')
end
if plot_flag(7+2)
    plot(T,P(:,7+2),'r--','DisplayName','yDot (m/s)')
end
if plot_flag(8+2)
    plot(T,P(:,8+2),'--','Color',[0 0.5 0],'DisplayName','phiDot (rad/s)')
end
if plot_flag(9+2)
    plot(T,P(:,9+2),'k--','DisplayName','alphaDot_R (rad/s)')
end
if plot_flag(10+2)
    plot(T,P(:,10+2),'m--','DisplayName','betaDot_R (rad/s)')
end
if plot_flag(13)
    plot(T,P(:,13),'k--','DisplayName','alphaDot_L (rad/s)')
end
if plot_flag(14)
    plot(T,P(:,14),'m--','DisplayName','betaDot_L (rad/s)')
end
if plot_flag(11+4)
    plot(T,P(:,11+4),'Color',[0.5 0 0],'DisplayName','L (m)')
    plot([T(1) T(end)],[param.L_sp0 param.L_sp0],'k--','DisplayName','L_{eq,R} (m)')
end
if plot_flag(12+4)
    plot(T,P(:,12+4),'--','Color',[0.5 0 0],'DisplayName','LDot_R (m/s)')
end
if plot_flag(13+4)
    plot(T,P(:,13+4),'--','Color',[0.8 0 0],'DisplayName','Energy (J)')
end
if plot_flag(14+4)
    plot(T,P(:,14+4),'--','Color',[0 0 0.8],'DisplayName','Desired Energy (J)')
end
if plot_flag(15+4)
    plot(T,P(:,15+4),'--','Color',[0.8 0 0],'DisplayName','tau_{hip,R} (N*m)')
end
if plot_flag(16+4)
    plot(T,P(:,16+4),'--','Color',[0 0 0.8],'DisplayName','tau_{knee,R} (N*m)')
end
if plot_flag(17+4)
    plot(T,P(:,17+4),'Color',[1 0 0],'DisplayName','F_cx (N)')
end
if plot_flag(18+4)
    plot(T,P(:,18+4),'Color',[0 0 1],'DisplayName','F_cy (N)')
end
if plot_flag(19+4)
    plot(T,P(:,19+4),'Color',[1 0 0],'DisplayName','theta (rad)')
end
if plot_flag(20+4)
    plot(T,P(:,20+4),'Color',[0 0 1],'DisplayName','dtheta (rad/s)')
end
if plot_flag(21+4)
    plot(T,P(:,21+4),'Color',[0 0 1],'DisplayName','RightFootPos_x (m)')
end
if plot_flag(22+4)
    plot(T,P(:,22+4),'Color',[0 0 1],'DisplayName','RightFootPos_y (m)')
end
if plot_flag(23+4)
    plot(T,P(:,23+4),'--','Color',[0.8 0 0],'DisplayName','tau_{hip,L} (N*m)')
end
if plot_flag(24+4)
    plot(T,P(:,24+4),'--','Color',[0 0 0.8],'DisplayName','tau_{knee,L} (N*m)')
end
hold off
