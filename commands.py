salloc --time=1:0:0 --mem-per-cpu=16G --ntasks=1 --gres=gpu:1
sbatch run_xarm6_training.sh --gpus-per-node=p100:1
sbatch run_xarm6_training.sh --gpus-per-node=p100:1
cat slurm-57986325.out

cp run_xarm6_training run_xarm6_training.sh

