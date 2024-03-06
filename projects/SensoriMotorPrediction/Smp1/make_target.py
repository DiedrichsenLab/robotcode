import pandas as pd

experiment = 'smp1'
subj = '100'
session = 'scanning'  # training or scanning
nblocks = 10

for block in range(nblocks):
    tgt = pd.read_csv(f'target/smp0_template_{session}.tgt', sep="\t")  # read template file

    go_rows = tgt['GoNogo'] == 'go'
    planTime_shuffled = tgt.loc[go_rows, 'planTime'].sample(frac=1).reset_index(drop=True).to_list()
    tgt.loc[go_rows, 'planTime'] = planTime_shuffled

    et = tgt.endTime  # extrapolate endTime
    st = tgt.startTimeDiff  # extrapolate startTimeDiff
    st1 = st.iloc[[0]]  # extrapolate first startTime
    st_rest = st.iloc[1:]  # extrapolate rest of startTimeDiff
    st_rest = st_rest.sample(frac=1).reset_index(drop=True)  # shuffle intervals
    st = pd.concat([st1, st_rest]).reset_index(drop=True)  # re-attach first sample

    # create startTime with shuffled intervals
    for s, d in enumerate(st.iloc[1:]):
        st.iloc[[s+1]] = d + st.iloc[[s]]

    tgt = tgt.sample(frac=1).reset_index(drop=True)  # shuffle template (without startTime)
    tgt = tgt.drop('startTimeDiff', axis=1)  # drop startTimeDiff column
    tgt['startTime'] = st  # add startTime to target
    tgt['endTime'] = et  # add endTime to target

    filename = f"{experiment}_{subj}_{block + 1:02}_{session}.tgt"

    tgt.to_csv('target/' + filename, index=False, sep='\t')

    print(filename)
