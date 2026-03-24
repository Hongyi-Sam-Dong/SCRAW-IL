set -ex

./compile.sh


MAP_PATH=maps/kiva_large_w_mode.json
AGENT_NUM=100
PORT_NUM=8182
SEED=0
SCREEN=0        
CUTOFF_TIME=1

./build/sillm -m $MAP_PATH \
                -a $AGENT_NUM \
                -p $PORT_NUM \
                -s $SEED \
                -sc $SCREEN \
                -c $CUTOFF_TIME