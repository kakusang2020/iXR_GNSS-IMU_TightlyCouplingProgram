function [quat] = eul2quat(eul)
	quat(1) = cos(eul(1) / 2.0)*cos(eul(2) / 2.0)*cos(eul(3) / 2.0) + sin(eul(1) / 2.0)*sin(eul(2) / 2.0)*sin(eul(3) / 2.0);
	quat(2) = sin(eul(1) / 2.0)*cos(eul(2) / 2.0)*cos(eul(3) / 2.0) - cos(eul(1) / 2.0)*sin(eul(2) / 2.0)*sin(eul(3) / 2.0);
	quat(3) = cos(eul(1) / 2.0)*sin(eul(2) / 2.0)*cos(eul(3) / 2.0) + sin(eul(1) / 2.0)*cos(eul(2) / 2.0)*sin(eul(3) / 2.0);
	quat(4) = cos(eul(1) / 2.0)*cos(eul(2) / 2.0)*sin(eul(3) / 2.0) - sin(eul(1) / 2.0)*sin(eul(2) / 2.0)*cos(eul(3) / 2.0);
end

