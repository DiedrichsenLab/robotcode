import pandas as pd

experiment = 'smp1'
subj = '100'
nblocks = 10

for block in range(nblocks):
    tgt = pd.read_csv('target/template.tgt', sep="\t")

    st = tgt.startTime
    st1 = st.iloc[[0]]
    st_rest = st.iloc[1:]
    st_rest = st_rest.sample(frac=1).reset_index(drop=True)
    st = pd.concat([st1, st_rest]).reset_index(drop=True)

    tgt = tgt.sample(frac=1).reset_index(drop=True)
    tgt.startTime = st

    filename = f"{experiment}_{subj}_{block + 1:02}.tgt"

    tgt.to_csv('target/' + filename, index=False, sep='\t')

    print(filename)
