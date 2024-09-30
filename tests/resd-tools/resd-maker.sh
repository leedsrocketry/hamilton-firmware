./csv2resd.py \
    -i accel_2.csv \
        -m acceleration:accel_x,accel_y,accel_z:x,y,z:0 \
        -s 0 \
        -f 1000 \
    output.resd

mv output.resd ../../renode/output.resd