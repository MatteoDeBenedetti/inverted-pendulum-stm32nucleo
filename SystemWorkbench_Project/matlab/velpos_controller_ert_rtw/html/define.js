function CodeDefine() { 
this.def = new Array();
this.def["rt_OneStep"] = {file: "ert_main_c.html",line:37,type:"fcn"};
this.def["main"] = {file: "ert_main_c.html",line:74,type:"fcn"};
this.def["V_in"] = {file: "velpos_controller_c.html",line:24,type:"var"};
this.def["velpos_controller_DW"] = {file: "velpos_controller_c.html",line:27,type:"var"};
this.def["velpos_controller_U"] = {file: "velpos_controller_c.html",line:30,type:"var"};
this.def["velpos_controller_M_"] = {file: "velpos_controller_c.html",line:33,type:"var"};
this.def["velpos_controller_M"] = {file: "velpos_controller_c.html",line:34,type:"var"};
this.def["velpos_controller_step"] = {file: "velpos_controller_c.html",line:37,type:"fcn"};
this.def["velpos_controller_initialize"] = {file: "velpos_controller_c.html",line:136,type:"fcn"};
this.def["velpos_controller_terminate"] = {file: "velpos_controller_c.html",line:160,type:"fcn"};
this.def["DW_velpos_controller_T"] = {file: "velpos_controller_h.html",line:47,type:"type"};
this.def["ExtU_velpos_controller_T"] = {file: "velpos_controller_h.html",line:53,type:"type"};
this.def["RT_MODEL_velpos_controller_T"] = {file: "velpos_controller_types_h.html",line:24,type:"type"};
this.def["int8_T"] = {file: "rtwtypes_h.html",line:51,type:"type"};
this.def["uint8_T"] = {file: "rtwtypes_h.html",line:52,type:"type"};
this.def["int16_T"] = {file: "rtwtypes_h.html",line:53,type:"type"};
this.def["uint16_T"] = {file: "rtwtypes_h.html",line:54,type:"type"};
this.def["int32_T"] = {file: "rtwtypes_h.html",line:55,type:"type"};
this.def["uint32_T"] = {file: "rtwtypes_h.html",line:56,type:"type"};
this.def["real32_T"] = {file: "rtwtypes_h.html",line:57,type:"type"};
this.def["real64_T"] = {file: "rtwtypes_h.html",line:58,type:"type"};
this.def["real_T"] = {file: "rtwtypes_h.html",line:64,type:"type"};
this.def["time_T"] = {file: "rtwtypes_h.html",line:65,type:"type"};
this.def["boolean_T"] = {file: "rtwtypes_h.html",line:66,type:"type"};
this.def["int_T"] = {file: "rtwtypes_h.html",line:67,type:"type"};
this.def["uint_T"] = {file: "rtwtypes_h.html",line:68,type:"type"};
this.def["ulong_T"] = {file: "rtwtypes_h.html",line:69,type:"type"};
this.def["char_T"] = {file: "rtwtypes_h.html",line:70,type:"type"};
this.def["uchar_T"] = {file: "rtwtypes_h.html",line:71,type:"type"};
this.def["byte_T"] = {file: "rtwtypes_h.html",line:72,type:"type"};
this.def["creal32_T"] = {file: "rtwtypes_h.html",line:82,type:"type"};
this.def["creal64_T"] = {file: "rtwtypes_h.html",line:87,type:"type"};
this.def["creal_T"] = {file: "rtwtypes_h.html",line:92,type:"type"};
this.def["cint8_T"] = {file: "rtwtypes_h.html",line:99,type:"type"};
this.def["cuint8_T"] = {file: "rtwtypes_h.html",line:106,type:"type"};
this.def["cint16_T"] = {file: "rtwtypes_h.html",line:113,type:"type"};
this.def["cuint16_T"] = {file: "rtwtypes_h.html",line:120,type:"type"};
this.def["cint32_T"] = {file: "rtwtypes_h.html",line:127,type:"type"};
this.def["cuint32_T"] = {file: "rtwtypes_h.html",line:134,type:"type"};
this.def["pointer_T"] = {file: "rtwtypes_h.html",line:152,type:"type"};
}
CodeDefine.instance = new CodeDefine();
var testHarnessInfo = {OwnerFileName: "", HarnessOwner: "", HarnessName: "", IsTestHarness: "0"};
var relPathToBuildDir = "../ert_main.c";
var fileSep = "\\";
var isPC = true;
function Html2SrcLink() {
	this.html2SrcPath = new Array;
	this.html2Root = new Array;
	this.html2SrcPath["ert_main_c.html"] = "../ert_main.c";
	this.html2Root["ert_main_c.html"] = "ert_main_c.html";
	this.html2SrcPath["velpos_controller_c.html"] = "../velpos_controller.c";
	this.html2Root["velpos_controller_c.html"] = "velpos_controller_c.html";
	this.html2SrcPath["velpos_controller_h.html"] = "../velpos_controller.h";
	this.html2Root["velpos_controller_h.html"] = "velpos_controller_h.html";
	this.html2SrcPath["velpos_controller_private_h.html"] = "../velpos_controller_private.h";
	this.html2Root["velpos_controller_private_h.html"] = "velpos_controller_private_h.html";
	this.html2SrcPath["velpos_controller_types_h.html"] = "../velpos_controller_types.h";
	this.html2Root["velpos_controller_types_h.html"] = "velpos_controller_types_h.html";
	this.html2SrcPath["rtwtypes_h.html"] = "../rtwtypes.h";
	this.html2Root["rtwtypes_h.html"] = "rtwtypes_h.html";
	this.getLink2Src = function (htmlFileName) {
		 if (this.html2SrcPath[htmlFileName])
			 return this.html2SrcPath[htmlFileName];
		 else
			 return null;
	}
	this.getLinkFromRoot = function (htmlFileName) {
		 if (this.html2Root[htmlFileName])
			 return this.html2Root[htmlFileName];
		 else
			 return null;
	}
}
Html2SrcLink.instance = new Html2SrcLink();
var fileList = [
"ert_main_c.html","velpos_controller_c.html","velpos_controller_h.html","velpos_controller_private_h.html","velpos_controller_types_h.html","rtwtypes_h.html"];
