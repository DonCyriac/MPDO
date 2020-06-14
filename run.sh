higher=10
seed=7
grid=10

python3 optimal_random.py $seed $higher $grid > /dev/null

while [[ $? -eq 1 ]]; do
    higher=$(($higher*2))
    python3 optimal_random.py $seed $higher $grid > /dev/null
done
# echo higher is $higher

lower=$(($higher/2))
while [[ lower -le higher ]]; do
    mid=$((($lower+$higher)/2))
    echo lower is $lower
    echo mid is $mid
    echo higher is $higher
    python3 optimal_random.py $seed $mid $grid > /dev/null
    if [[ $? -eq 1 ]]
    then
        lower=$(($mid+1))
    else
        higher=$(($mid-1))
    fi
done
echo optimal is $(($mid+1))
