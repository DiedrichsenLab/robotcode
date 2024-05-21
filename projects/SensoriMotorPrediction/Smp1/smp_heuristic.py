import os
import numpy
from cfmm_base import infotodict as cfmminfodict
from cfmm_base import create_key


def infotodict(seqinfo):
    """Heuristic evaluator for determining which runs belong where
    allowed template fields - follow python string module:
    item: index within category
    subject: participant id
    seqitem: run number during scanning
    subindex: sub index within group
    """

    # call cfmm for general labelling and get dictionary
    info = cfmminfodict(seqinfo)

    task = create_key('{bids_subject_session_dir}/func/{bids_subject_session_prefix}_task-task_run-{item:02d}_bold')
    task_sbref = create_key(
        '{bids_subject_session_dir}/func/{bids_subject_session_prefix}_task-task_run-{item:02d}_sbref')

    fmap_sbref = create_key('{bids_subject_session_dir}/fmap/{bids_subject_session_prefix}_dir-{dir}_epi')

    # t13d = create_key('{bids_subject_session_dir}/anat/{bids_subject_session_prefix}_acq-MPRAGE_run-{item:02d}_T1w')

    info[task] = []
    info[task_sbref] = []
    info[fmap_sbref] = []
    # info[t13d]=[]

    for idx, s in enumerate(seqinfo):

        if ('2.3mm_A-P' in (s.series_description).strip()):
            if (s.dim4 == 1 and 'SBRef' in (s.series_description).strip()):
                info[task_sbref].append({'item': s.series_id})
            elif (s.dim4 > 300):
                info[task].append({'item': s.series_id})

        elif ('2.3mm_P-A' in (s.series_description).strip()):
            if (s.dim4 == 1):
                # if 'SBRef' in (s.series_description).strip():
                info[fmap_sbref].append({'item': s.series_id, 'dir': 'PA'})

        # elif ('mp2rage' in (s.series_description).strip()):
        #    info[t13d].append({'item': s.series_id})

    return info
