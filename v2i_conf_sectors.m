function [BSLocX,LocLaneY,sector_limits] = v2i_conf_sectors(BEAMWIDTH,TOT_COV_ANGLE,BSLOC,LANELENGTH,LANEID)
    LocLaneY = BSLOC + (LANELENGTH/2) + (LANEID-1)*LANELENGTH;
    degrees = (180-TOT_COV_ANGLE)/2 + (0:BEAMWIDTH:TOT_COV_ANGLE);
    x = LocLaneY ./ tand(-degrees);
    sector_limits = x - max(abs(x));
    BSLocX = -max(abs(sector_limits))/2;
end