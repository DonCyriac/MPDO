higher=18
seed=7
grid=10
program="optimal_random.py"

python3 $program $seed $grid $higher > /dev/null

while [[ $? -eq 1 ]]; do
    higher=$(($higher*2))
    python3 $program $seed $grid $higher > /dev/null
done

lower=$(($higher/2))
while [[ lower -le higher ]]; do
    mid=$((($lower+$higher)/2))
    echo lower is $lower
    echo mid is $mid
    echo higher is $higher
    python3 $program $seed $grid $mid  > /dev/null
    if [[ $? -eq 1 ]]
    then
        lower=$(($mid+1))
    else
        higher=$(($mid-1))
    fi
done
echo optimal is $(($mid+1))
