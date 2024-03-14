
# python modules
import argparse
import importlib
import os.path
import sys

# project files
import meteorite
import arena
from turret import Turret
import runner
import cases

from text_display import TextRunnerDisplay
try:
    from turtle_display import TurtleRunnerDisplay
except ImportError as e:
    sys.stderr.write('turtle display not available, using text instead\n')
    TurtleRunnerDisplay = lambda h, w: TextRunnerDisplay()

TURRET_INITIAL_POS = {'x': 0.0,
                      'y': -1.0}


def display_for_name(dname):
    """Set up desired display tupe."""
    if dname == 'turtle':
        return TurtleRunnerDisplay(800, 800)
    elif dname == 'text':
        return TextRunnerDisplay()
    else:
        return runner.RunnerDisplay()


def case_params(case_num):
    """Get the parameters for the given case."""
    return cases.index[case_num]


def run_method(method_name):
    """Convert input method into the function needed to run that method."""
    if method_name == 'estimate':
        return runner.run_estimation
    elif method_name == 'defense':
        return runner.run_defense
    elif method_name == 'kf_nonoise':
        return runner.run_kf_nonoise
    else:
        raise RuntimeError('unknown method %s' % method_name)


def run_kwargs(params):
    """Set up kwargs for running main."""
    meteorites = []
    for themeteorite in params['meteorites']:
        meteorites.append(
            meteorite.Meteorite(themeteorite,
                                params['accel_corr_factor_s'],
                                params['laser_effectiveness_distance']))

    in_bounds = arena.Arena()
    turret = Turret(TURRET_INITIAL_POS,
                    params['_args']['max_angle_change'],
                    params['dt'])

    ret = {'field': meteorite.MeteorShower(in_bounds, params['_args']['seed'],
                                           params['prob_hit_destroys'],
                                           params['laser_effectiveness_distance'],
                                           meteorites, turret,
                                           params['min_dist']),
           'in_bounds': in_bounds,
           'noise_sigma_x': params['noise_sigma_x'],
           'noise_sigma_y': params['noise_sigma_y'],
           'min_dist': params['min_dist'],
           'turret': turret,
           'turret_init_health': params['initial_laser_state']['hp'],
           'num_laser_shots': params['num_laser_shots'],
           'max_angle_change': params['_args']['max_angle_change'],
           'nsteps': params['_args']['nsteps'],
           'dt': params['_args']['dt'],
           'seed': params['_args']['seed']}

    return ret


def main(method_name, case_id, display_name):
    """Run the specified case using the specified method."""
    try:
        params = cases.index[int(case_id)]
    except Exception as e:
        try:
            mdl_name = os.path.splitext(os.path.split(case_id)[1])[0]
            mdl = importlib.import_module(mdl_name)
            params = mdl.params
        except Exception as e:
            print(e)
            return

    import timeit
    start = timeit.default_timer()
    retcode, t = run_method(method_name)(display=display_for_name(display_name),
                                         **(run_kwargs(params)))
    stop = timeit.default_timer()
    print(f'Approximate run time: {stop - start} seconds')
    print((retcode, t))


def parser():
    """Parse command-line arguments."""
    prsr = argparse.ArgumentParser()
    prsr.add_argument('method',
                      help="Which method to test",
                      type=str,
                      choices=('kf_nonoise', 'estimate', 'defense'),
                      default='estimate')
    prsr.add_argument('--case',
                      help="test case number (one of %s) or test case file" % list(cases.index.keys()),
                      type=str,
                      default=1)
    prsr.add_argument('--display',
                      choices=('turtle', 'text', 'none'),
                      default='none')
    return prsr


if __name__ == '__main__':
    args = parser().parse_args()
    if 'none' in args.display:
        gui_runcom = sys.argv
        gui_runcom.insert(-1, '--display turtle')
        thecommand = '    python ' + ' '.join(gui_runcom)
        print('No display method provided in run command; defaulting to \'text\'.')
        print('To re-run this simulation with the GUI visualization, please run the command\n')
        print(thecommand + '\n')
        args.display = 'text'
    main(method_name=args.method,
         case_id=args.case,
         display_name=args.display)
