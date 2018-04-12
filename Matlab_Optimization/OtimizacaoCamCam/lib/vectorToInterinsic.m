function [ intrinsics ] = vectorToInterinsic( intrinsic_vector )
%vectorToInterinsic Get a interinsic matrix with the values of the vector

intrinsics = [intrinsic_vector(1) 0 intrinsic_vector(3) 0
    0 intrinsic_vector(2) intrinsic_vector(4) 0
    0 0 1 0];

end