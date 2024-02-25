#uses https://github.com/AtsushiSakai/pycubicspline
import pycubicspline.pycubicspline as pyspline
from trajectory import Trajectory
from map import Map

import numpy as np
import yaml
import math
from matplotlib import pyplot 
import random

#x = np.linspace(0, 10, num=11)
#y = np.cos(-x**2 / 9.)
#spl = CubicSpline(x, y)

#karte holen.
# scp "192.168.64.5:/home/wette/cartographer_ws/my_map*" .

class RacelineOptimizer:
    def __init__(self, configfile: str):
        self.__config = None
        
        self.parse_config(configfile)
        self.__map = Map(self.__config["image"], self.__config["origin"], self.__config["resolution"])

    def get_map(self) -> Map:
        return self.__map

    def get_config(self):
        return self.__config

    def debug_draw_map(self):
        pyplot.imshow(self.__map.get_pixel_map())
        pyplot.show()

    def debug_draw_trajectory(self, trajectory : Trajectory, filename: str = None):
        pyplot.imshow(self.__map.get_pixel_map())

        #add the second point of the spine as last, too -> nicer circle
        xs = trajectory.x[:]
        xs.append(trajectory.x[1])
        ys = trajectory.y[:]
        ys.append(trajectory.y[1])
        lx,ly, _, _, _ = pyspline.calc_2d_spline_interpolation(xs, ys, num=300)

        rl = Trajectory(lx, ly, trajectory.haftreibung, trajectory.vehicle_width_m, trajectory.vehicle_acceleration_mss, trajectory.vehicle_deceleration_mss, trajectory.resolution)
        rl.do_forwards_pass = True
        rl.compute_velocity_profile()
        print(f"Raceline with 300 points time: {rl.get_laptime()}")

        pyplot.scatter(rl.x,rl.y, c=rl.velocity_profile, linewidth=1, cmap=pyplot.cm.coolwarm)
        pyplot.colorbar()
        #pyplot.plot(lx,ly)
        #pyplot.scatter(trajectory.x, trajectory.y, marker="x")
        if filename is not None:
            pyplot.savefig(filename)
            pyplot.clf()
        else:
            pyplot.show()

        

    def get_manual_initial_centerline(self, num_control_points = 200) -> (int, int):
        print("#####################")
        print("#### click in the image to give manual controlpoints to a spline")
        print("#####################")
        fig = pyplot.figure()
        ax = fig.add_subplot()
        ax.imshow(self.__map.get_pixel_map())
        points = []

        def onclick(event):
            print('add point x=%f, y=%f' % (event.xdata, event.ydata))
            p = [int(event.xdata), int(event.ydata)]
            ax.plot([event.xdata],[event.ydata],"bo")
            fig.canvas.draw()
            points.append(p)

        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        pyplot.show()

        #remove all points which are not within free space
        for p in points[:]:
            if self.__map[p[0]][p[1]] > 0.0:
                points.remove(p)

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]

        #start and end with the same point.
        xs.append(xs[0])
        ys.append(ys[0])

        #interpolate:
        x, y, yaw, k, travel = pyspline.calc_2d_spline_interpolation(xs, ys, num=num_control_points)
        
        print(f"Initial Lap Length [m]: {max(travel) * self.__config['resolution']}")


        return x, y

    def parse_config(self, filename: str):
        f = open(filename, 'r')
        self.__config = yaml.safe_load(f)
        f.close()

    def optimize_raceline(self, initial_trajectory: Trajectory, num_epochs=250, num_keep=20, num_population=200, max_change_in_pixels=3, num_changes_per_mutation=1) -> Trajectory:
        """use a genetic algorithm to find a raceline"""

        def remove_all_but_top(population: list, num_keep: int):
            #population.sort(key=lambda x : -1 * x.get_length()) #-1 to sort descending!
            population.sort(key=lambda x : -1 * x.get_laptime()) #-1 to sort descending!
            return population[len(population)-num_keep : len(population)]

        #initialize population with random racelines deduced from the initial trajectory.
        population = []
        for i in range(num_population):
            t = initial_trajectory.copy()
            t.random_changes(max_change_in_pixels, num_changes_per_mutation, self.__map)
            population.append(t)
        population.append(initial_trajectory.copy()) #keep in the original one w/o modifications

        #remove all but the top racelines
        population = remove_all_but_top(population, num_keep)
        
        #go through epochs
        for e in range(0, num_epochs):

            #create new offspring
            new_childs = []
            offspring_count = int(math.ceil(num_population/num_keep))
            for rl in population:
                for i in range(offspring_count):
                    new_childs.append(rl.copy())

            #mutate offspring
            for rl in new_childs:
                rl.random_changes(max_change_in_pixels, num_changes_per_mutation, self.__map)
            
            new_childs += population[:] #copy over old trajectories to new childs for random_combination
            
            #combine
            combined_childs = []
            random.shuffle(new_childs)
            for i in range(0, int(len(new_childs)/2)):
                i = random.randint(0, len(new_childs)-1)
                j = random.randint(0, len(new_childs)-1)
                combined = new_childs[i].copy()
                combined.random_combination(new_childs[j])
                combined_childs.append( combined )
            
            #add new children to population:
            for l in new_childs:
                population.append(l)
            for l in combined_childs:
                population.append(l)

            #make sure population are NOT driving through non-free space!
            vehicle_width_in_map_pixels = math.ceil(population[0].vehicle_width_m / self.__config['resolution'])
            for l in population[:]:
                lx,ly, _, _, _ = pyspline.calc_2d_spline_interpolation(l.x, l.y, num=500)
                #connect first with last point of the spline
                #x2,y2,_,_,_    = pyspline.calc_2d_spline_interpolation([l.x[-1], l.x[0]], [l.y[-1], l.y[0]], num=30)
                #lx +=x2
                #ly +=y2

                #for each point of the trajectory:
                for i in range(len(lx)):
                    #check if a square around each point of the trajectory is all in free space
                    #TODO: This should actually be a circle!
                    removeTrajectory = False
                    if self.__map[int(ly[i])][int(lx[i])] > 0.0:
                        #print(f"Point {i} not in free space: {int(lx[i])},{int(ly[i])}")
                        removeTrajectory = True
                        break
                    for dx in range(-math.floor(vehicle_width_in_map_pixels/2.0), math.ceil(vehicle_width_in_map_pixels/2.0), 1):
                        for dy in range(-math.floor(vehicle_width_in_map_pixels/2.0), math.ceil(vehicle_width_in_map_pixels/2.0), 1):
                            if self.__map[int(ly[i]+dy)][int(lx[i]+dx)] > 0.0:
                                #print(f"Point {i} not in free space: {int(ly[i]+dy)},{int(lx[i]+dx)}")
                                removeTrajectory = True
                                break
                        if removeTrajectory:
                            break
                    if removeTrajectory:
                            break

                if removeTrajectory:
                    population.remove(l)

            if len(population) == 0:
                raise Exception("Not possible to find a raceline. Check parameters and initial raceline.")
            
            print(f"Valid Population size: {len(population)}")

            #remove all but the top racelines
            population = remove_all_but_top(population, num_keep)

            #print length of best raceline.
            print(f"raceline length/laptime in epoch {e}: {population[-1].get_length()} / {population[-1].get_laptime()}")
            self.debug_draw_trajectory(population[-1], f"racelines/racelineEpoch{e}.png")
            

        return population[-1]
    

    def test(self, t: Trajectory):
        self.debug_draw_trajectory(t)
        idx = 0
        x = t.x[idx]
        y = t.y[idx]
        #test different changes - keep the first one which is in free space
        normalx, normaly = t.compute_normal_vector(idx)
        change = 10

        t.x[idx] = x + change*normalx
        t.y[idx] = y + change*normaly
        #if first point is moved, last point needs to move, too!
        if idx == 0:
            t.x[len(t.x)-1] = t.x[idx]
            t.y[len(t.y)-1] = t.y[idx]
                
        #apply spline smoothing
        t.x, t.y, _, t.curvature, _ = pyspline.calc_2d_spline_interpolation(t.x, t.y, num=len(t.y))
        t.length = None
        t.laptime = None

        self.debug_draw_trajectory(t)

def main():
    haftreibung                 = 0.1
    vehicle_width_m             = 0.2   #half width is minimum distance to any wall at any time
    vehicle_acceleration_mss    = 5.0   #vehicle acceleration in meter/sec/sec
    vehicle_deceleration_mss    = 10.0  #vehicle deceleration in meter/sec/sec

    desired_points_per_meter    = 0.4   #how many control points to use during optimization per spline (you want as few as possible!)
    max_change_per_point_meters = 0.15   #how much change to a controlpoint per iteration in meters (should be pretty small; few cm)

    num_epochs                  = 40     #number of optimization epochs
    num_keep                    = 3      #number of trajectories to keep after each epoch
    num_population              = 200    #population size during epoch
    num_changes_per_mutation    = 2      #number of controlpoint changes during a mutation

    opt = RacelineOptimizer("my_map.yaml")
    
    #for development: fixed start trajectory - in production this should come from waypoints sampled from follow the gap algorithm.
    #x,y = opt.get_manual_initial_centerline()
    x = [202.0, 208.17146036653043, 214.34226033242697, 220.51173949705577, 226.67924995032297, 232.8445433925939, 239.0078457481098, 245.1694104915597, 251.3294910976325, 257.4883410410173, 263.646213796403, 269.8033628384787, 275.9600416419332, 282.1165036814556, 288.2730024317349, 294.42978961253414, 300.58704651141545, 306.74486162289224, 312.90331710753367, 319.06249512590904, 325.22247783858757, 331.38334740613845, 337.5451859891311, 343.7080757481345, 349.8720988437181, 356.03733743645114, 362.20387368690274, 368.3717897349596, 374.54105520695487, 380.7112638644422, 386.8819310754577, 393.052572207977, 399.2226019811742, 405.39104267803305, 411.5568209484751, 417.71886344242193, 423.8760968097951, 430.0274477005162, 436.1718427645067, 442.30820865168835, 448.43547249327656, 454.5542034173197, 460.6702822337132, 466.7906618520658, 472.9222951819864, 479.07213513308375, 485.2471346149667, 491.4537548731639, 497.6787316763258, 503.8828168969174, 510.0249891881547, 516.0643567731032, 521.9971917035094, 527.9080424756322, 533.8942792551355, 540.0528615919063, 546.3451911043292, 552.401981801964, 557.804471812251, 562.1339048675799, 565.1010607838917, 566.8813616213654, 567.7543701702117, 567.9996492206413, 567.8858896770784, 567.6172228359177, 567.3740424599022, 567.3367056275645, 567.6855694174374, 568.6009909080532, 570.2366873219332, 572.4796365945521, 575.0631435923938, 577.7186653427506, 580.1776588729149, 582.1782084021411, 583.5493191346774, 584.185998185611, 583.984735841917, 582.8931744977713, 581.0495207004557, 578.6363488799569, 575.8362334662617, 572.8145478577942, 569.5694901178214, 566.0051719994527, 562.0246539652529, 557.5424327048848, 552.5690432989155, 547.163025381012, 541.3832741584013, 535.2886848383104, 528.938152627966, 522.3905727345956, 515.7048403654255, 508.9398507276828, 502.1544990285944, 495.40768047538745, 488.7582896247965, 482.2505754401177, 475.87630532957525, 469.61549562695325, 463.44816266603607, 457.35432278060796, 451.31399230445317, 445.3071875713558, 439.31392491510024, 433.3142206694708, 427.2880911682517, 421.21620571452434, 415.09774660082627, 408.95258782015486, 402.80169376859914, 396.6660276513996, 390.5579549825622, 384.4606274689484, 378.3509976892521, 372.2075411636233, 366.0300937482618, 359.8342337680331, 353.6359060110841, 347.4510552655613, 341.2956263196114, 335.1855639613809, 329.1347402306586, 323.1304392956683, 317.1415291724817, 311.13650887906465, 305.08425239735277, 298.979332063132, 292.8581108338654, 286.760785584694, 280.7274200004664, 274.77376003736487, 268.86310426755523, 262.9519268960415, 256.9967021278277, 250.9545964228927, 244.82336936382472, 238.66316401996917, 232.5394276381428, 226.5176074651623, 220.59985138928167, 214.50665231982606, 208.28584216489548, 202.09137004792112, 195.92580924187072, 189.78287877637825, 183.65628589184362, 177.53776323176052, 171.41487127197362, 165.27464045636242, 159.10410122880637, 152.890284033185, 146.62606152977642, 140.41752911509175, 134.47337037730452, 129.00597772343073, 124.22744757506298, 120.26004594237232, 117.01032695235538, 114.35313086628169, 112.16329794542082, 110.31651816263624, 108.72397077124681, 107.34441071822965, 106.13992522607813, 105.07260151728562, 104.10452681434542, 103.19778833975086, 102.3219333542306, 101.50593105605707, 100.80715917270027, 100.28317219914248, 99.99152134228598, 99.96217160784117, 100.13056971885383, 100.41193572572142, 100.72148967884137, 100.97445162861113, 101.1375870576018, 101.4900573767318, 102.42809394604419, 104.34813445284486, 107.59104487558703, 112.02933958552612, 117.30072821366262, 123.04116148181801, 128.92244392653026, 134.852456508574, 140.83455109079395, 146.8723478992798, 152.9694671601203, 159.12952909940506, 165.35152457955192, 171.606087378029, 177.85311885800323, 184.05250018285304, 190.16411251595702, 196.14783755079327]
    y = [389.0, 388.95837465065927, 388.9380717882851, 388.9604138998441, 389.04633266907257, 389.2042567928682, 389.4277774814041, 389.70962394818457, 390.0425254067138, 390.4192110704959, 390.83241015303497, 391.27485186783514, 391.7392654284006, 392.2183800482355, 392.7049249408439, 393.19164879903684, 393.6720820981652, 394.14078529680665, 394.5923891589787, 395.0215244486984, 395.42282192998346, 395.79091236685116, 396.1204265233188, 396.405995163404, 396.642249051124, 396.8238189504962, 396.945335625538, 397.0014315953824, 396.99628717243894, 396.96597819742396, 396.953232910475, 397.00077954653375, 397.1427284390548, 397.3795881659319, 397.70367886866495, 398.1073206887542, 398.58283376769975, 399.12253824700167, 399.7187542681602, 400.3638019726754, 401.05000021706473, 401.76528397402296, 402.48340682706254, 403.1752600152036, 403.8117347774661, 404.3637223528702, 404.80211398043593, 405.09747928103764, 405.2074846118021, 405.072799170917, 404.63293221930695, 403.8275440124881, 402.63960377883524, 401.1549539509986, 399.47437873033175, 397.69834462298667, 395.8224363776085, 393.5863843106096, 390.69163903456274, 386.83965627242867, 381.8499979955713, 375.96586992628556, 369.52542948366647, 362.86683408680915, 356.290539573853, 349.87312164509586, 343.6088388606242, 337.4918225668334, 331.51620411011857, 325.676114836875, 319.962043487329, 314.3280061506505, 308.7070063974404, 303.0317951336297, 297.23512326514896, 291.2566761421856, 285.13127541183144, 278.9628046367121, 272.85669832987963, 266.8927347053684, 261.05511095109824, 255.3057707110443, 249.6066576291817, 243.93684940744285, 238.44194817225477, 233.36127602755718, 228.93520227455016, 225.37882317131513, 222.69499868737358, 220.7805030062599, 219.531324525617, 218.8434516430879, 218.6128727563156, 218.735576262943, 219.10755056061313, 219.62478404696898, 220.1832651196535, 220.67898217630963, 221.0079247483257, 221.09160822997305, 220.9430183259799, 220.59562173531586, 220.08288515695048, 219.43827528985338, 218.69525883299414, 217.88730248534233, 217.0478729458675, 216.21043691353935, 215.40846108732737, 214.67417597993867, 214.00476374369714, 213.35823348562488, 212.69052998701096, 211.95760039507408, 211.13247339717665, 210.24621847394673, 209.3422212794792, 208.46434545889846, 207.6631588211553, 206.99417011286764, 206.5130030989689, 206.27528154439256, 206.33662921407188, 206.75266987294034, 207.57230347219976, 208.7581812415327, 210.21321402957517, 211.839115687498, 213.53673258971818, 215.14745813285103, 216.41600337668243, 217.07820963689298, 216.87093494141655, 215.71666745901655, 213.93825410144117, 211.91063580018505, 210.00875348674276, 208.602707174047, 207.7787303693363, 207.18681070221479, 206.43984384612298, 205.15072547450137, 203.33373061578047, 202.7846878743738, 203.18335484978923, 203.52219349288978, 203.762205648063, 203.9210494888459, 204.01648746935845, 204.0837481586039, 204.19496464986756, 204.42695838531614, 204.85655080711635, 205.56056335743492, 206.61717724061637, 208.13092598756867, 210.2302202904324, 213.04433406024353, 216.70224438039645, 221.24284231295084, 226.48869323468566, 232.23055841573841, 238.25919912624673, 244.3675571646277, 250.43964685345645, 256.4815711700896, 262.50798437099024, 268.5335407126211, 274.57289445144534, 280.6406998439261, 286.74901608249456, 292.88923172386535, 299.0428530761658, 305.1913249568186, 311.3160934634806, 317.4093455577694, 323.5000694994855, 329.62512893285714, 335.8213875021122, 342.12570885147886, 348.54788127990855, 354.93360041574175, 361.0670683822615, 366.7323789249238, 371.7316343707501, 376.01871118315444, 379.62357670711555, 382.5767682801498, 384.9163846381393, 386.73031190785986, 388.12657057357603, 389.21323771607985, 390.0983904161631, 390.8901057546178, 391.6687709226658, 392.34515918473227, 392.76584937423263, 392.7772995023768, 392.2259675803752, 390.95831422608046]
    
    #add the first point of the spine as last, too -> circle
    x.append(x[0])
    y.append(y[0])
    x,y, _, _, path_len = pyspline.calc_2d_spline_interpolation(x, y, num=100)
    
    #compute number of control points:

    track_in_meters = path_len[-1] * opt.get_config()["resolution"]
    num_ctrl_points = math.ceil(track_in_meters * desired_points_per_meter)

    #resample spline with desired number of control points:
    x,y, _, _, path_len = pyspline.calc_2d_spline_interpolation(x, y, num=num_ctrl_points)

    #opt.debug_draw_trajectory(Trajectory(x,y))
    

    #use genetic algorithm to optimize x,y
    original = Trajectory(x, y, haftreibung, vehicle_width_m, vehicle_acceleration_mss, vehicle_deceleration_mss, opt.get_config()["resolution"])
    #opt.test(original)
    raceline = opt.optimize_raceline(initial_trajectory=original, num_epochs=num_epochs, num_keep=num_keep, num_population=num_population, 
                                    num_changes_per_mutation=num_changes_per_mutation, 
                                    max_change_in_pixels=max_change_per_point_meters/opt.get_config()["resolution"])

    raceline.safe_trajectory_to_file("my_map_raceline.csv", num_points=300)

    raceline.laptime = None
    raceline.do_forwards_pass = True
    print(f"Raceline time: {raceline.get_laptime()}")
    opt.debug_draw_trajectory(raceline)

    


if __name__ == "__main__":
    main()
    #test()