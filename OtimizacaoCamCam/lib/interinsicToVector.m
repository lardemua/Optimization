function [ intrinsic_vector ] = interinsicToVector( intrinsics )
%interinsicToVector Get a vector with the values of the interinsic matrix 
%that will be iterated

intrinsic_vector = [intrinsics(1,1) intrinsics(2,2) intrinsics(1,3) intrinsics(2,3)];
 

end

