namespace ProjectRiseForm
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.getDataButton = new System.Windows.Forms.Button();
            this.dataDisplayBox = new System.Windows.Forms.RichTextBox();
            this.startVisualizationButton = new System.Windows.Forms.Button();
            this.ConnectButton = new System.Windows.Forms.Button();
            this.tbNumOfScans = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.SuspendLayout();
            // 
            // getDataButton
            // 
            this.getDataButton.Location = new System.Drawing.Point(187, 24);
            this.getDataButton.Name = "getDataButton";
            this.getDataButton.Size = new System.Drawing.Size(75, 23);
            this.getDataButton.TabIndex = 0;
            this.getDataButton.Text = "Get Data";
            this.getDataButton.UseVisualStyleBackColor = true;
            this.getDataButton.Click += new System.EventHandler(this.getDataButton_Click);
            // 
            // dataDisplayBox
            // 
            this.dataDisplayBox.Location = new System.Drawing.Point(50, 91);
            this.dataDisplayBox.Name = "dataDisplayBox";
            this.dataDisplayBox.Size = new System.Drawing.Size(318, 178);
            this.dataDisplayBox.TabIndex = 3;
            this.dataDisplayBox.Text = "";
            // 
            // startVisualizationButton
            // 
            this.startVisualizationButton.Location = new System.Drawing.Point(250, 276);
            this.startVisualizationButton.Name = "startVisualizationButton";
            this.startVisualizationButton.Size = new System.Drawing.Size(121, 23);
            this.startVisualizationButton.TabIndex = 4;
            this.startVisualizationButton.Text = "Start Visualization";
            this.startVisualizationButton.UseVisualStyleBackColor = true;
            this.startVisualizationButton.Click += new System.EventHandler(this.startVisualizationButton_Click);
            // 
            // ConnectButton
            // 
            this.ConnectButton.Location = new System.Drawing.Point(187, 62);
            this.ConnectButton.Name = "ConnectButton";
            this.ConnectButton.Size = new System.Drawing.Size(75, 23);
            this.ConnectButton.TabIndex = 5;
            this.ConnectButton.Text = "Connect";
            this.ConnectButton.UseVisualStyleBackColor = true;
            this.ConnectButton.Click += new System.EventHandler(this.ConnectButton_Click);
            // 
            // tbNumOfScans
            // 
            this.tbNumOfScans.Location = new System.Drawing.Point(81, 62);
            this.tbNumOfScans.Name = "tbNumOfScans";
            this.tbNumOfScans.Size = new System.Drawing.Size(100, 20);
            this.tbNumOfScans.TabIndex = 6;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(84, 43);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(62, 13);
            this.label2.TabIndex = 7;
            this.label2.Text = "NoOfScans";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(396, 307);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.tbNumOfScans);
            this.Controls.Add(this.ConnectButton);
            this.Controls.Add(this.startVisualizationButton);
            this.Controls.Add(this.dataDisplayBox);
            this.Controls.Add(this.getDataButton);
            this.Name = "Form1";
            this.Text = "Form1";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button getDataButton;
        private System.Windows.Forms.RichTextBox dataDisplayBox;
        private System.Windows.Forms.Button startVisualizationButton;
        private System.Windows.Forms.Button ConnectButton;
        private System.Windows.Forms.TextBox tbNumOfScans;
        private System.Windows.Forms.Label label2;
    }
}

