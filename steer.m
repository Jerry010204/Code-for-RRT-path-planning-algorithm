% For the use of HKU MECH3433 Robotics, drones and autonomous ground vehicles.
% 'qr' and 'qn' are coordinate of two node; 'val' is the distance between them.
% 'eps' is the threhold you are going to use.
function A = steer(qr, qn, val, eps)
   % Steer towards qn with maximum step size of eps
    if val > eps
        %qnew = qn + eps/val;
        %fprintf('%d\n', qn(1));
        %fprintf('%d\n', qn(2));
        %fprintf('%d\n', val);
        qnew = qn.coord+(qr-qn.coord)*eps/(val);
        %fprintf('%d\n', qnew(1));
        %fprintf('%d\n', qnew(2));
    else
        qnew = qr;

    end
    
    A = [qnew(1), qnew(2)];
end
