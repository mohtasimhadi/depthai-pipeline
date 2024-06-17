import os

with open('.env', 'r') as fh:
    vars_dict = dict(
        tuple(line.replace('\n', '').split('='))
        for line in fh.readlines() if not line.startswith('#')
    )

print(vars_dict)
os.environ.update(vars_dict)

CAM_1 = os.getenv("CAM_1_MXID")

print(CAM_1)