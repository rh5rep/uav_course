function rosmsgOut = ArmRequest(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.arm = logical(slBusIn.arm);
end
