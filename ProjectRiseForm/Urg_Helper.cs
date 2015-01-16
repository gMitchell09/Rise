using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ProjectRiseForm
{
    static class Native_Urg_Helper
    {
        [DllImport(@"Urg_Helper.dll")]
        public static extern void PassStructIn(ref MyStruct theStruct);
    }
}
