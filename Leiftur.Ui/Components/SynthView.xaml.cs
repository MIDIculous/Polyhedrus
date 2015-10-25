﻿using System.Windows;
using System.Windows.Controls;

namespace Leiftur.Ui.Components
{
	/// <summary>
	/// Interaction logic for SynthView.xaml
	/// </summary>
	public partial class SynthView : UserControl
	{
		private SynthViewModel Vm;

		public SynthView()
		{
			InitializeComponent();
		}

		private void Button_Click(object sender, RoutedEventArgs e)
		{
			var controls = OscAddress.GetChildrenWithValue(this);
			this.Vm = new SynthViewModel(controls);
			this.DataContext = Vm;
			Vm.RequestState();
		}
	}
}
