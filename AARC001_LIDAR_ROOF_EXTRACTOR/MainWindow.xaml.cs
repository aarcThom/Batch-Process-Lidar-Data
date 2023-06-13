using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Forms;
using LasSharp;
using Path = System.IO.Path;
using MessageBox = System.Windows.MessageBox;

namespace AARC001_LIDAR_ROOF_EXTRACTOR
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public string? InputPath { get; set; } = null;
        public string? OutputPath { get; set; } = null;
        public string? CloudPath { get; set; } = null;
        public int CloudMin { get; set; } = 500;
        public int OctLevel { get; set; } = 10;
        public float PercentStep { get; set; }
        public float Percentage { get; set; } = 0f;
        public int XyMin { get; set; } = 100;

        //the rounded extents of all LAS files
        public int RndXmin;

        public int RndXmax;

        public int RndYmin;

        public int RndYmax;


        //getting the extents of all lidar files
        public List<double> LasExtents = new List<double>();

        public string? TempDirectory;


        public MainWindow()
        {
            InitializeComponent();
        }

        private void ccPath_Click(object sender, RoutedEventArgs e)
        {
            using var dialog = new OpenFileDialog();
            dialog.Filter = "exe files (*.exe)|*.exe";
            dialog.RestoreDirectory = true;

            if (dialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                FileInfo cc = new FileInfo(dialog.FileName);

                if (cc.Name == "CloudCompare.exe")
                {
                    CloudPath = cc.DirectoryName;
                    ccPathText.Text = cc.Name;
                }
                else
                {
                    MessageBox.Show("This isn't a valid cloud compare exe!");
                }
                
            }
        }

        private void inputButton_Click(object sender, RoutedEventArgs e)
        {
            using var dialog = new FolderBrowserDialog();
            if (dialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                InputPath = dialog.SelectedPath;
                inputBox.Text = dialog.SelectedPath;
            }
        }

        private void outputButton_Click(object sender, RoutedEventArgs e)
        {
            using var dialog = new FolderBrowserDialog();
            if (dialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                OutputPath = dialog.SelectedPath;
                outputBox.Text = dialog.SelectedPath;
            }

        }

        private void runButton_Click(object sender, RoutedEventArgs e)
        {
            if (InputPath != null && OutputPath != null && CloudPath != null)
            {

                string[] lasFiles =
                    Directory.GetFiles(InputPath, "*.las", SearchOption.AllDirectories);

                feedback.Text = "Searching directories";
                System.Windows.Forms.Application.DoEvents();

                //figuring out the maximum extents of the ALL LIDAR files.
                if (lasFiles.Length != 0)

                {
                    foreach (var lasFile in lasFiles)
                    {
                        LasReader las = new LasReader(lasFile);
                        List<double> fileExtents = new List<double> { las.MinX, las.MaxX, las.MinY, las.MaxY };

                        if (LasExtents.Count == 0)
                        {
                            LasExtents.AddRange(fileExtents);
                        }
                        else
                        {
                            for (int i = 0; i < 4; i++)
                            {
                                if (i % 2 == 0)
                                {
                                    if (fileExtents[i] < LasExtents[i])
                                    {
                                        LasExtents[i] = fileExtents[i];
                                    }
                                }
                                else
                                {
                                    if (fileExtents[i] > LasExtents[i])
                                    {
                                        LasExtents[i] = fileExtents[i];
                                    }
                                }
                            }
                        }
                    }

                    //breaking the las extents into segments

                    RndXmin = RoundExtents(LasExtents[0], XyMin, "min");

                    RndXmax = RoundExtents(LasExtents[1], XyMin, "max");

                    RndYmin = RoundExtents(LasExtents[2], XyMin, "min");

                    RndYmax = RoundExtents(LasExtents[3], XyMin, "max");

                    // writing the dictionary and the directory / files
                    for (int i = RndXmin; i < RndXmax + 1; i = i + XyMin)
                    {
                        string dirPath = OutputPath + @"/" + i.ToString();
                        Directory.CreateDirectory(dirPath);

                        for (int j = RndYmin; j < RndYmax + 1; j = j + XyMin)
                        {
                            string filePath = dirPath + @"/" + j + ".asc";
                            var newFile = File.Create(filePath);
                            newFile.Close();

                        }
                    }
                }


                if (lasFiles.Length != 0)
                {
                    float steps = Convert.ToSingle(lasFiles.Length) * 5;
                    PercentStep = 100f / steps;

                    //creating the dictionary for points
                    Dictionary<int, Dictionary<int, List<string>>> pointDict =
                        new Dictionary<int, Dictionary<int, List<string>>>();

                    for (int lasCount = 0; lasCount < lasFiles.Length; lasCount++)
                    {
                        if (pointDict.Count == 0)
                        {
                            for (int i = RndXmin; i < RndXmax + 1; i = i + XyMin)
                            {
                                pointDict.Add(i, new Dictionary<int, List<string>>());

                                for (int j = RndYmin; j < RndYmax + 1; j = j + XyMin)
                                {
                                    pointDict[i].Add(j, new List<string>());
                                }
                            }
                        }
                        


                        //picking the proper octree divisions
                        List<double> lasDims = new List<double>();
                        LasReader currentLas = new LasReader(lasFiles[lasCount]);
                        lasDims.Add(Math.Abs(currentLas.MaxX - currentLas.MinX));
                        lasDims.Add(Math.Abs(currentLas.MaxY - currentLas.MinY));
                        lasDims.Sort();
                        double maxVal = lasDims[1];
                        double tempVal = Math.Log(maxVal / OctLevel) / Math.Log(2);
                        int octoStep = Convert.ToInt32(Math.Round(tempVal));

                        feedback.Text = "First Pass: File " + (lasCount + 1).ToString() + " of " + lasFiles.Length.ToString();
                        Percentage = Percentage + PercentStep;
                        progBar.Value = Percentage;
                        System.Windows.Forms.Application.DoEvents();

                        //adding in the event output to see what's going on. See https://stackoverflow.com/a/4291965

                        Process firstPass = new Process
                        {
                            StartInfo = new ProcessStartInfo
                            {
                                FileName = "cmd.exe",
                                Arguments = "/c CloudCompare -SILENT -FWF_O " + lasFiles[lasCount] +
                                            " -AUTO_SAVE OFF -FILTER_SF 6 6 " +
                                            "-OCTREE_NORMALS 1.5 -ORIENT PLUS_Z -MODEL TRI -NORMALS_TO_SFS " +
                                            "-SET_ACTIVE_SF LAST -FILTER_SF 0.7 1 -SOR 100 1 -EXTRACT_CC "
                                            + octoStep.ToString() + " " + CloudMin.ToString() + " -REMOVE_ALL_SFS "
                                            + "-NO_TIMESTAMP -C_EXPORT_FMT ASC -ADD_PTS_COUNT -SAVE_CLOUDS",
                                WorkingDirectory = CloudPath,
                                RedirectStandardOutput = true,
                                UseShellExecute = false,
                                CreateNoWindow = true
                            }
                        };

                        // creating the debug file
                        if (!File.Exists(OutputPath + @"/debugLog.txt"))
                        {
                            var debug = File.CreateText(OutputPath + @"/debugLog.txt");
                            debug.Close();
                        }
                        var debugWrite = new StreamWriter(OutputPath + @"/debugLog.txt", true);

                        debugWrite.WriteLine("________________________CLOUD "+ (lasCount + 1).ToString()+"____________________");

                        firstPass.Start();
                        while (!firstPass.StandardOutput.EndOfStream)
                        {
                            string outLine = firstPass.StandardOutput.ReadLine();
                            debugWrite.WriteLine(outLine);
                        }

                        firstPass.WaitForExit();
                        debugWrite.Close();

                        feedback.Text = "Creating Temp Files: File " + (lasCount + 1).ToString() + " of " + lasFiles.Length.ToString();
                        Percentage = Percentage + PercentStep;
                        progBar.Value = Percentage;
                        System.Windows.Forms.Application.DoEvents();

                        //creating the temp directory
                        TempDirectory = OutputPath + @"/temp";
                        if (!(Directory.Exists(TempDirectory)))
                        {
                            Directory.CreateDirectory(TempDirectory);
                        }
                        else
                        {
                            DirectoryInfo di = new DirectoryInfo(TempDirectory);
                            FileInfo[] files = di.GetFiles();
                            foreach (FileInfo file in files)
                            {
                                file.Delete();
                            }
                        }

                        feedback.Text = "Removing Small Clouds: File " + (lasCount + 1).ToString() + " of " + lasFiles.Length.ToString();
                        Percentage = Percentage + PercentStep;
                        progBar.Value = Percentage;
                        System.Windows.Forms.Application.DoEvents();

                        //getting the paths for all the derivative point clouds >= to set min pt count
                        string lasFilePath = Path.GetDirectoryName(lasFiles[lasCount]);
                        string[] newLasFiles = Directory.GetFiles(lasFilePath, "*.asc", SearchOption.TopDirectoryOnly);
                        List<string> derivLas = new List<string>();

                        int counter = 0;
                        foreach (var lasPath in newLasFiles)
                        {
                            int ptCount = Int32.Parse(File.ReadLines(lasPath).First());
                            if (ptCount >= CloudMin)
                            {
                                string newFile = TempDirectory + @"/" + counter.ToString() + ".asc";
                                File.Move(lasPath, newFile);
                                derivLas.Add(newFile);
                                counter++;
                            }
                            else
                            {
                                File.Delete(lasPath);
                            }
                        }

                        //sorting the points
                        feedback.Text = "Sorting Points: File " + (lasCount + 1).ToString() + " of " + lasFiles.Length.ToString();
                        Percentage = Percentage + PercentStep;
                        progBar.Value = Percentage;
                        System.Windows.Forms.Application.DoEvents();


                        //writing the points from the temp asc files to the proper files
                        string[] ascFiles;
                        ascFiles = Directory.GetFiles(TempDirectory, "*.asc", SearchOption.TopDirectoryOnly);
                        int ascCount = 1;
                        int allAsc = ascFiles.Length;

                        foreach (var ascFile in ascFiles)
                        {
                            feedback.Text = "Sorting Points: File " + (lasCount + 1).ToString() + " of "
                                            + lasFiles.Length.ToString() + " - subfile " + ascCount.ToString() +
                                            " of " + allAsc.ToString();
                            System.Windows.Forms.Application.DoEvents();

                            using (var reader = new StreamReader(ascFile))
                            {
                                //ignore first line
                                string? line = reader.ReadLine();

                                while (!reader.EndOfStream)
                                {
                                    line = reader.ReadLine();

                                    string[] coords = line.Split(' ');

                                    int xPath = RoundExtents(Double.Parse(coords[0]), XyMin, "min");
                                    int yFile = RoundExtents(Double.Parse(coords[1]), XyMin, "min");

                                    pointDict[xPath][yFile].Add(line);
                                }
                                reader.Close();
                                ascCount++;
                            }
                        }

                        //writing to dictionary every 10th loop or the last file

                        if ((lasCount+1) % 10 == 0 | lasCount == lasFiles.Length - 1)
                        {
                            feedback.Text = "Writing to sorted ASC files.";
                            System.Windows.Forms.Application.DoEvents();

                            foreach (var xVal in pointDict)
                            {
                                string xKey = xVal.Key.ToString();

                                foreach (var yVal in xVal.Value)
                                {
                                    string yKey = yVal.Key.ToString();

                                    string ptPath = OutputPath + @"/" + xKey + @"/" + yKey + ".asc";
                                    File.AppendAllLines(ptPath, yVal.Value);
                                }
                            }

                            //clearing the dictionary
                            pointDict.Clear();
                        }

                        


                        feedback.Text = "Cleaning up: File " + (lasCount + 1).ToString() + " of " + lasFiles.Length.ToString();
                        Percentage = Percentage + PercentStep;
                        progBar.Value = Percentage;
                        System.Windows.Forms.Application.DoEvents();

                        //delete the unsorted asc files
                        for (int i = 0; i < ascFiles.Length; i++)
                        {
                            File.Delete(ascFiles[i]);
                        }


                    }

                    //deleting zero length asc files
                    string[] outFiles;
                    outFiles = Directory.GetFiles(OutputPath, "*.asc", SearchOption.AllDirectories);

                    foreach (var outfile in outFiles)
                    {
                        FileInfo fInfo = new FileInfo(outfile);
                        if (fInfo.Length < 10)
                        {
                            File.Delete(outfile);
                        }
                    }


                    //deleting temp directory and empty point directories
                    string[] outDir;

                    outDir = Directory.GetDirectories(OutputPath);

                    foreach (var dir in outDir)
                    {
                        DirectoryInfo dInfo = new DirectoryInfo(dir);
                        if (dInfo.EnumerateFiles().Count() == 0)
                        {
                            Directory.Delete(dir);
                        }
                    }



                    feedback.Text = "Done! You may exit.";
                    Percentage = 100;
                    progBar.Value = Percentage;
                    System.Windows.Forms.Application.DoEvents();

                }
                else
                {
                    MessageBox.Show("There aren't any .las files in this directory!");
                }
            }
            else if (CloudPath == null)
            {
                MessageBox.Show("You need to specify Cloud Compare's location.");
            }

            else if (InputPath == null && OutputPath != null)
            {
                MessageBox.Show("Select an input path.\nWhere are the LAS files located?");
            }
            else if (OutputPath == null && InputPath != null)
            {
                MessageBox.Show("Select an output path.\nWhere do you want to output your files?");
            }
            else
            {
                MessageBox.Show("Select input and output paths, ya dingus.");
            }
        }

        private void ChangeFbText(object sender, RoutedEventArgs e, string text)
        {
            feedback.Text = text;
        }

        private void MinSegPts_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (int.TryParse(minSegPts.Text, out int tempInt))
            {
                CloudMin = tempInt;
            }
        }

        private void OctreeCell_TextChanged(object sender, TextChangedEventArgs e)
        {

            if (int.TryParse(octreeCell.Text, out int tempInt))
            {
                OctLevel = tempInt;
            }
        }

        private void XyDim_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (int.TryParse(xyDim.Text, out int tempInt))
            {
                XyMin = tempInt;
            }
        }

        private int RoundExtents(double valIn, int rndIn, string minmax)
        {

            int rndOut = 0;

            if (minmax == "min")
            {
                rndOut = (int)Math.Round((valIn / (double)rndIn), MidpointRounding.ToZero) *
                             rndIn;
                if (rndOut > valIn)
                {
                    rndOut -= rndIn;
                }
            }

            if (minmax == "max")
            {
                rndOut = (int)Math.Round((valIn / (double)rndIn), MidpointRounding.AwayFromZero) *
                             rndIn;
                if (rndOut < valIn)
                {
                    rndOut += rndIn;
                }
            }

            return rndOut;
        }

        
    }
}

