function [beamwidth] = v2i_config_BSbeamwidthEL(ang_max, LocLaneY, h, n_lanes)
    d_mid=sqrt(LocLaneY^2+h^2);
    dmax=d_mid/cosd(ang_max/2);
    %d_mid=dmax*cosd(ang_max/2);
    %l=sqrt(d_mid^2-h^2);
    d=LocLaneY-3.5*n_lanes;

    alpha = atand(d/h);
    beta=acosd(h/dmax);

    beamwidth = beta - alpha;
end