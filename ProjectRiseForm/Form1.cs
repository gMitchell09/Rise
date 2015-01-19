using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ManagedUrgHelper;

namespace ProjectRiseForm
{
    public partial class Form1 : Form
    {
        ManagedUrgHelper.ManagedUrgHelper helper;

        public Form1()
        {
            helper = new ManagedUrgHelper.ManagedUrgHelper();
            InitializeComponent();           
        }

        private void ConnectButton_Click(object sender, EventArgs e)
        {
            if (!helper.ConnectToUrg())
                MessageBox.Show("Unable to connect Check com");
        }

        private void getDataButton_Click(object sender, EventArgs e)
        {
            //string numOfScans = tbNumOfScans.Text;
            //try
            //{
            //    List<int> data = helper.GetDataFromTheUrg(Convert.ToDouble(numOfScans));
            //    foreach (int theData in data)
            //        dataDisplayBox.AppendText(theData.ToString() + " ");
            //    dataDisplayBox.AppendText("\n");
            //}
            //catch (Exception ex)
            //{
            //    throw;
            //}
        }

        private void startVisualizationButton_Click(object sender, EventArgs e)
        {
            helper.StartCloudVisualization();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            //helper.Dispose();
        }

  
    }
}
