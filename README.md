# ATLASCAR2 Sensors Calibration by Global Optimization
Contains the code of the for the Laboratory of Automation and Robotics, Department of Mechanical Engineering, University of Aveiro (**LARDEMUA**).

Check the github page [github.com/lardemua](https://github.com/lardemua) or the [lars.mec.ua.pt](http://lars.mec.ua.pt).

![docs/guidelines-readme.png](docs/guidelines-readme.png?raw=true "guidelines")

# Table of Contents

- [Guidelines for LARDEMUA](#guidelines-for-lardemua)
- [Table of Contents](#table-of-contents)
- [Installation](#installation)
- [Usage](#usage)
- [Credits](#credits)
- [License](#license)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

# Installation

If this is a python standalone project, so just use the pip install and the [requirements.txt file](https://pip.readthedocs.io/en/1.1/requirements.html) mechanism:

```bash
sudo pip install -r requirements.txt
```

# Usage

## Lemonbot datasets
To use the datasets taken from the lemonbot camera, go to the OpenConstructorOptimization folder 

```bash
cd OpenConstructorOptimization
```

and run

```bash
./Optimization.py ../CameraImages/DataSet1 center all fromaruco -do -ms 0.082
```

All the arguments are:

```bash
usage: Optimization.py [-h] [-d] [-no] [-do] [-saveResults] [-processDataset]
                       [-ms marksize]
                       Directory {center,corners} {all,translation}
                       {fromaruco,fromfile}

positional arguments:
  Directory             Directory of the dataset to optimize
  {center,corners}      Chose if is the centers or the corners of Arucos to do
                        the otimization.
  {all,translation}     Chose if use translation and rotation or only the
                        translations in the vector x to otimize.
  {fromaruco,fromfile}  Get initial estimation from the markers detection or
                        from a file.

optional arguments:
  -h, --help            show this help message and exit
  -d                    draw initial estimation
  -no                   do not optimize
  -do                   to draw during optimization
  -saveResults          Save results of the optimization
  -processDataset       Process the point clouds with the results obtained
                        from the optimization process
  -ms marksize          size of the aruco markers (m)

```

## OpenConstructor datasets



# Credits

List the authors, current develpers, inspiring projects, etc.

Here goes an example of a table with the authors:

Author Name  | Email
------------- | -------------
Filipe Costa | costa.filipe@ua.pt
Miguel Riem Oliveira | mriem@ua.pt
Vitor Santos | vitor@ua.pt

# License
GNU GP3. See LICENSE for full details.
