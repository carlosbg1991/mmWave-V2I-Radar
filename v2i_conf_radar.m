function [error,t_sector,n_slots] = v2i_conf_radar(NPKTRadar,BEAMRadar,BEAMWIDTH,TSLOT)
    if NPKTRadar==10
        error = 2;
        t_radar_uni = 1.25;
    else
        error = 32;
        t_radar_uni = 0.072;
    end
    t_sector = (BEAMWIDTH/BEAMRadar) * t_radar_uni;
    n_slots = ceil(TSLOT/t_sector);
end