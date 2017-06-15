function [] = plotEigenValues( eigenV, V )
%PLOT_EIGEN_VALUES plot eigen values of the bicycle
% Inputs : 
%   eigenV = vector of eigen values
%   V = vector of velocities

    eig1 = eigenV(1,:);
    eig2 = eigenV(2,:);
    eig3 = eigenV(3,:);
    eig4 = eigenV(4,:);
    
    figure()
    
    for itr = 1 : length(V)
        v = V(itr);
        if isreal(eig1(itr))
            hold on
            plot(v, eig1);
        else
            hold on
            i = imag(eig(itr));
            plot(v, i, v,-i)
        end
    end
    hold off



end

