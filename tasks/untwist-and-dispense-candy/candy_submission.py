"""Generate the board fields for the revised candy from its measured demo."""


INSTRUCTION = (
    'Untwist one end of the wrapper and let the candy fall free onto the tabletop. '
    'Leave it intact and clear of the wrapper.'
)
CONTEXT = (
    'Opening a portion for someone else. Holding the wrapper gives the robot a way '
    'to dispense the sweet and keep the loose food away from its hands.'
)
SCORING = (
    'Score 0 if the wrapper tears, the candy chips or breaks, the candy touches the '
    'floor or an unclean surface, or either hand touches the candy after it has '
    'fully left the wrapper. Otherwise 0.25 for holding an end and untwisting '
    'it relative to the wrapper body by at least 90 degrees for at least 0.25 s '
    'before the candy comes out. Use the printed stripe in the close video. '
    'Raise to 0.5 if the whole candy finishes outside the film with at least '
    '2 mm of clearance. Award 1.0 if it drops at least 3 cm onto the clean tabletop '
    'from the wrapper without being caught, and rests there for the last two '
    'seconds with at least 2 mm clearance. During that hold the hands must be '
    'open at least 75 mm and each pinch point at least 8 cm from the candy '
    'center and the wrapper. All other outcomes score 0. Use close video '
    'from reset to release and a ruler in the final photograph.'
)


def task_record(grade):
    score = grade.get('task_score')
    scores = 'Revised scripted demo pending verification.' if score is None else f'Scripted demo: {score:.2f}.'
    return {
        'title': 'Untwist and dispense a wrapped candy',
        'category': 'Societal relatability',
        'keywords': ['bimanual', 'twist wrapper', 'flexible packaging', 'controlled release'],
        'instruction': INSTRUCTION,
        'context': CONTEXT,
        'setup': (
            'One individually wrapped molded hard candy, 17 mm in diameter, 14 mm high '
            'and about 3.4 g. One 128 by 67 mm strip of printed twist-retaining '
            'cellophane, approximately 45 micrometres thick, overlaps underneath '
            'the candy. Both ends are twisted at least half a turn and have a printed '
            'registration stripe. Put it midway between the two fixed YAM arms and '
            '34 cm forward of their base line, with its long axis parallel to the '
            'base line. The arm bases are 61 cm apart. The clean rigid laminate worktop is '
            '110 by 130 cm and 76 cm above the floor. Both grippers start open in '
            'the ready pose. The calibrated cameras and their fixed mount are the '
            'only other equipment. Match the setup photograph.'
        ),
        'scoring': SCORING,
        'action_limit': 2400,
        'run_limit': '2,400 control steps (120 seconds at 20 Hz).',
        'reset_notes': (
            'Use a fresh intact sweet and wrapper with the same dimensions and twists. '
            'Place the overlap seam down, align the stripe and tails to the setup '
            'photograph, clean the worktop, and reset both arms. Verify on a separate '
            'hands-off trial of the full duration that neither end untwists 90 '
            'degrees by itself and the sweet stays inside.'
        ),
        'fidelity': (
            'Native rigid candy and a continuous deformable film with self-contact '
            'and gripper contact. The rounded visual mesh lies inside the convex '
            'collision hull. A formed rest mesh approximates retained twists and '
            'springback. Film bending and contact friction are approximate. '
            'The model has fixed topology and a rigid sweet, so it does not model '
            'tears, fracture, or sticky residue. Inspect these on the physical item.'
        ),
        'scores': scores + ' Dream-1 pending.',
    }
