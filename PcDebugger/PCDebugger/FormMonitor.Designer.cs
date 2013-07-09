namespace PCDebugger
{
	partial class FormMonitor
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
			this.components = new System.ComponentModel.Container();
			this._graphAngularVelocity = new ZedGraph.ZedGraphControl();
			this._graphAccelAngle = new ZedGraph.ZedGraphControl();
			this._graphAngle = new ZedGraph.ZedGraphControl();
			this.SuspendLayout();
			// 
			// _graphAngularVelocity
			// 
			this._graphAngularVelocity.Location = new System.Drawing.Point(16, 15);
			this._graphAngularVelocity.Margin = new System.Windows.Forms.Padding(5);
			this._graphAngularVelocity.Name = "_graphAngularVelocity";
			this._graphAngularVelocity.ScrollGrace = 0D;
			this._graphAngularVelocity.ScrollMaxX = 0D;
			this._graphAngularVelocity.ScrollMaxY = 0D;
			this._graphAngularVelocity.ScrollMaxY2 = 0D;
			this._graphAngularVelocity.ScrollMinX = 0D;
			this._graphAngularVelocity.ScrollMinY = 0D;
			this._graphAngularVelocity.ScrollMinY2 = 0D;
			this._graphAngularVelocity.Size = new System.Drawing.Size(540, 312);
			this._graphAngularVelocity.TabIndex = 0;
			// 
			// _graphAccelAngle
			// 
			this._graphAccelAngle.Location = new System.Drawing.Point(565, 15);
			this._graphAccelAngle.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
			this._graphAccelAngle.Name = "_graphAccelAngle";
			this._graphAccelAngle.ScrollGrace = 0D;
			this._graphAccelAngle.ScrollMaxX = 0D;
			this._graphAccelAngle.ScrollMaxY = 0D;
			this._graphAccelAngle.ScrollMaxY2 = 0D;
			this._graphAccelAngle.ScrollMinX = 0D;
			this._graphAccelAngle.ScrollMinY = 0D;
			this._graphAccelAngle.ScrollMinY2 = 0D;
			this._graphAccelAngle.Size = new System.Drawing.Size(540, 312);
			this._graphAccelAngle.TabIndex = 1;
			// 
			// _graphAngle
			// 
			this._graphAngle.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
			this._graphAngle.Location = new System.Drawing.Point(211, 336);
			this._graphAngle.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
			this._graphAngle.Name = "_graphAngle";
			this._graphAngle.ScrollGrace = 0D;
			this._graphAngle.ScrollMaxX = 0D;
			this._graphAngle.ScrollMaxY = 0D;
			this._graphAngle.ScrollMaxY2 = 0D;
			this._graphAngle.ScrollMinX = 0D;
			this._graphAngle.ScrollMinY = 0D;
			this._graphAngle.ScrollMinY2 = 0D;
			this._graphAngle.Size = new System.Drawing.Size(701, 439);
			this._graphAngle.TabIndex = 2;
			// 
			// FormMonitor
			// 
			this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
			this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
			this.ClientSize = new System.Drawing.Size(1121, 788);
			this.Controls.Add(this._graphAngle);
			this.Controls.Add(this._graphAccelAngle);
			this.Controls.Add(this._graphAngularVelocity);
			this.Margin = new System.Windows.Forms.Padding(4);
			this.Name = "FormMonitor";
			this.Text = "Segway Monitor";
			this.Load += new System.EventHandler(this.FormMonitor_Load);
			this.ResumeLayout(false);

		}

		#endregion

		private ZedGraph.ZedGraphControl _graphAngularVelocity;
		private ZedGraph.ZedGraphControl _graphAccelAngle;
		private ZedGraph.ZedGraphControl _graphAngle;
	}
}

