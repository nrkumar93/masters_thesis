function kd_tree_obj = kDtree(xyt_data)
    kd_tree_obj = createns(xyt_data, 'Distance', @distfun);
end

function D2 = distfun(ZI, ZJ)

D2 = zeros(size(ZJ,1),1);

for i=1:size(ZJ,1)
    D2(i) = sqrt((ZI(1)-ZJ(i,1))^2 + (ZI(2)-ZJ(i,2))^2 + (angle_dist(ZI(3),ZJ(i,3)))^2);
end
end

function del = angle_dist(src, target)
    del = target - src;
    if del > pi
        del = del - 2*pi;
    elseif del < -pi
        del = del + 2*pi;
    end
end  
