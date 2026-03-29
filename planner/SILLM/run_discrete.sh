EXEC_FP=build/discrete_driver

MAP_FP="../../data/map/small/warehouse_small.map"
NUM_AGENTS=1
RANDOM_SEED=0
SIM_STEPS=64
PLAN_TIME_LIMIT=1.0

LNS_PLAN_WINDOW=15
LNS_EXEC_WINDOW=1
LNS_NUM_THREADS=0
LNS_MAX_ITER=-1

./${EXEC_FP} --map_fp ${MAP_FP} --num_agents ${NUM_AGENTS} --random_seed ${RANDOM_SEED} \
    --sim_steps ${SIM_STEPS} --plan_time_limit ${PLAN_TIME_LIMIT} \
    --lns_plan_window ${LNS_PLAN_WINDOW} --lns_exec_window ${LNS_EXEC_WINDOW} \
    --lns_num_threads ${LNS_NUM_THREADS} --lns_max_iter ${LNS_MAX_ITER}