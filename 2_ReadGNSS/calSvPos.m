function [svPos] = calSvPos(time,obs,nav,obsHeader)

switch obs.constellation
    case 'GPS'
        %% constant
        c = 299792458;                  % speed of light
        mu = 3.986005e+14;              % Earth's universal gravitation parameter
        OMEGA_dot_e = 7.2921151467e-5;  % Earth's rotation rate
        %% calculate satellite position
        A = nav.roota^2;
        n_0 = sqrt(mu / A^3);
        if isfield(obsHeader.obsType.typeIndexGPS,'C1C') && ~isnan(obs.measurements(obsHeader.obsType.typeIndexGPS.C1C))
            pseudorange = obs.measurements(obsHeader.obsType.typeIndexGPS.C1C);
        else
            svPos = [];
            return
        end
        t = time.GPST - pseudorange / c;
        t_k = t - nav.toe;
        if(t_k > 302400)
            t_k = t_k - 604800;
        elseif(t_k < -302400)
            t_k = t_k + 604800;
        end
        n = n_0 + nav.deltan;
        M_k = nav.M0 + n * t_k;
        E_k = KeplerSolve(nav.ecc,M_k,0.0000000001);
        
        sin_v_k = (sqrt(1-nav.ecc^2) * sin(E_k)) / (1 - nav.ecc * cos(E_k));
        cos_v_k = (cos(E_k)-nav.ecc) / (1 - nav.ecc * cos(E_k));
        v_k = atan2(sin_v_k,cos_v_k);
        PHI_k = v_k + nav.omega;
        delta_u_k = nav.cus * sin(2*PHI_k) + nav.cuc * cos(2*PHI_k);
        delta_r_k = nav.crs * sin(2*PHI_k) + nav.crc * cos(2*PHI_k);
        delta_i_k = nav.cis * sin(2*PHI_k) + nav.cic * cos(2*PHI_k);
        u_k = PHI_k + delta_u_k;
        r_k = nav.roota.^2 * (1-nav.ecc*cos(E_k)) + delta_r_k;
        i_k = nav.i0 + delta_i_k + nav.idot * t_k;
        x_k_aps = r_k * cos(u_k);
        y_k_aps = r_k * sin(u_k);
        
        OMEGA_k = nav.Omega0 + (nav.Omegadot - OMEGA_dot_e) * t_k - OMEGA_dot_e * nav.toe - OMEGA_dot_e * (pseudorange / c);
        svPos(1,1) = x_k_aps * cos(OMEGA_k) - y_k_aps * cos(i_k) * sin(OMEGA_k);
        svPos(2,1) = x_k_aps * sin(OMEGA_k) + y_k_aps * cos(i_k) * cos(OMEGA_k);
        svPos(3,1) = y_k_aps * sin(i_k);
    case 'QZSS'
        %% constant
        c = 299792458;                  % speed of light
        mu = 3.986005e+14;              % Earth's universal gravitation parameter
        OMEGA_dot_e = 7.2921151467e-5;  % Earth's rotation rate
        %% calculate satellite position
        A = nav.roota^2;
        n_0 = sqrt(mu / A^3);
        if isfield(obsHeader.obsType.typeIndexGPS,'C1C') && ~isnan(obs.measurements(obsHeader.obsType.typeIndexGPS.C1C))
            pseudorange = obs.measurements(obsHeader.obsType.typeIndexGPS.C1C);
        else
            svPos = [];
            return
        end
        t = time.GPST - pseudorange / c;
        t_k = t - nav.toe;
        if(t_k > 302400)
            t_k = t_k - 604800;
        elseif(t_k < -302400)
            t_k = t_k + 604800;
        end
        n = n_0 + nav.deltan;
        M_k = nav.M0 + n * t_k;
        E_k = KeplerSolve(nav.ecc,M_k,0.0000000001);
        
        sin_v_k = (sqrt(1-nav.ecc^2) * sin(E_k)) / (1 - nav.ecc * cos(E_k));
        cos_v_k = (cos(E_k)-nav.ecc) / (1 - nav.ecc * cos(E_k));
        v_k = atan2(sin_v_k,cos_v_k);
        PHI_k = v_k + nav.omega;
        delta_u_k = nav.cus * sin(2*PHI_k) + nav.cuc * cos(2*PHI_k);
        delta_r_k = nav.crs * sin(2*PHI_k) + nav.crc * cos(2*PHI_k);
        delta_i_k = nav.cis * sin(2*PHI_k) + nav.cic * cos(2*PHI_k);
        u_k = PHI_k + delta_u_k;
        r_k = nav.roota.^2 * (1-nav.ecc*cos(E_k)) + delta_r_k;
        i_k = nav.i0 + delta_i_k + nav.idot * t_k;
        x_k_aps = r_k * cos(u_k);
        y_k_aps = r_k * sin(u_k);
        
        OMEGA_k = nav.Omega0 + (nav.Omegadot - OMEGA_dot_e) * t_k - OMEGA_dot_e * nav.toe - OMEGA_dot_e * (pseudorange / c);
        svPos(1,1) = x_k_aps * cos(OMEGA_k) - y_k_aps * cos(i_k) * sin(OMEGA_k);
        svPos(2,1) = x_k_aps * sin(OMEGA_k) + y_k_aps * cos(i_k) * cos(OMEGA_k);
        svPos(3,1) = y_k_aps * sin(i_k);
    case 'BEIDOU'
        %% constant
        c = 299792458;                  % speed of light
        mu = 3.986004418e+14;              % Earth's universal gravitation parameter
        OMEGA_dot_e = 7.2921150e-5;  % Earth's rotation rate
        %% calculate satellite position
        A = nav.roota^2;
        n_0 = sqrt(mu / A^3);
        if isfield(obsHeader.obsType.typeIndexBEIDOU,'C1I') && ~isnan(obs.measurements(obsHeader.obsType.typeIndexBEIDOU.C1I))
            pseudorange = obs.measurements(obsHeader.obsType.typeIndexBEIDOU.C1I);
        elseif isfield(obsHeader.obsType.typeIndexBEIDOU,'C1Q') && ~isnan(obs.measurements(obsHeader.obsType.typeIndexBEIDOU.C1Q))
            pseudorange = obs.measurements(obsHeader.obsType.typeIndexBEIDOU.C1Q);
        elseif isfield(obsHeader.obsType.typeIndexBEIDOU,'C1X') && ~isnan(obs.measurements(obsHeader.obsType.typeIndexBEIDOU.C1X))
            pseudorange = obs.measurements(obsHeader.obsType.typeIndexBEIDOU.C1X);
        else
            svPos = [];
            return
        end
        t = time.GPST - 14 - pseudorange / c;
        
        t_k = t - nav.toe;
        if(t_k > 302400)
            t_k = t_k - 604800;
        elseif(t_k < -302400)
            t_k = t_k + 604800;
        end
        n = n_0 + nav.deltan;
        M_k = nav.M0 + n * t_k;
        E_k = KeplerSolve(nav.ecc,M_k,0.0000000001);
        
        sin_v_k = (sqrt(1-nav.ecc^2) * sin(E_k)) / (1 - nav.ecc * cos(E_k));
        cos_v_k = (cos(E_k)-nav.ecc) / (1 - nav.ecc * cos(E_k));
        v_k = atan2(sin_v_k,cos_v_k);
        PHI_k = v_k + nav.omega;
        delta_u_k = nav.cus * sin(2*PHI_k) + nav.cuc * cos(2*PHI_k);
        delta_r_k = nav.crs * sin(2*PHI_k) + nav.crc * cos(2*PHI_k);
        delta_i_k = nav.cis * sin(2*PHI_k) + nav.cic * cos(2*PHI_k);
        u_k = PHI_k + delta_u_k;
        r_k = nav.roota.^2 * (1-nav.ecc*cos(E_k)) + delta_r_k;
        i_k = nav.i0 + delta_i_k + nav.idot * t_k;
        x_k_aps = r_k * cos(u_k);
        y_k_aps = r_k * sin(u_k);
        
        if obs.svPRN <= 5 || obs.svPRN >= 59 % BEIDOU GEO
            OMEGA_k = nav.Omega0 + nav.Omegadot * t_k - OMEGA_dot_e * nav.toe;
            X_GK = x_k_aps * cos(OMEGA_k) - y_k_aps * cos(i_k) * sin(OMEGA_k);
            Y_GK = x_k_aps * sin(OMEGA_k) + y_k_aps * cos(i_k) * cos(OMEGA_k);
            Z_GK = y_k_aps * sin(i_k);
            
            R_X = [1,0,0;0,cosd(-5),sind(-5);0,-sind(-5),cosd(-5)];
            phi = OMEGA_dot_e * t_k + OMEGA_dot_e * pseudorange / c;
            
            R_Z = [cos(phi),sin(phi),0;-sin(phi),cos(phi),0;0,0,1];
            svPos = R_Z * R_X * [X_GK;Y_GK;Z_GK];
        else
            OMEGA_k = nav.Omega0 + (nav.Omegadot - OMEGA_dot_e) * t_k - OMEGA_dot_e * nav.toe - OMEGA_dot_e * (pseudorange / c);
            svPos(1,1) = x_k_aps * cos(OMEGA_k) - y_k_aps * cos(i_k) * sin(OMEGA_k);
            svPos(2,1) = x_k_aps * sin(OMEGA_k) + y_k_aps * cos(i_k) * cos(OMEGA_k);
            svPos(3,1) = y_k_aps * sin(i_k);
        end
end

end