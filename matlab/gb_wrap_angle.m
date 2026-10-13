function a = gb_wrap_angle(a)
%GB_WRAP_ANGLE  角度归一化到 [-pi, pi]（与 Python world.wrap_angle 逐行等价）。
    while a >  pi, a = a - 2*pi; end
    while a < -pi, a = a + 2*pi; end
end
