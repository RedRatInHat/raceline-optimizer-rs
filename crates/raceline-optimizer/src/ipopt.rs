#[cfg(not(feature = "ipopt-static-link"))]
use std::ffi::c_int;
use std::ffi::{c_char, c_void, CString};
use std::path::{Path, PathBuf};

const IPOPT_LIBRARY_ENV: &str = "RLC_IPOPT_LIBRARY";

#[cfg(windows)]
const DEFAULT_IPOPT_LIBRARY: &str = "libipopt-3.dll";

#[cfg(target_os = "macos")]
const DEFAULT_IPOPT_LIBRARY: &str = "libipopt.dylib";

#[cfg(all(not(windows), not(target_os = "macos")))]
const DEFAULT_IPOPT_LIBRARY: &str = "libipopt.so";

pub(crate) fn default_library_path(explicit_path: Option<PathBuf>) -> PathBuf {
    explicit_path
        .or_else(|| std::env::var_os(IPOPT_LIBRARY_ENV).map(PathBuf::from))
        .unwrap_or_else(|| PathBuf::from(DEFAULT_IPOPT_LIBRARY))
}

pub(crate) type IpoptProblem = *mut c_void;
pub(crate) type EvalF = unsafe extern "C" fn(i32, *mut f64, bool, *mut f64, *mut c_void) -> bool;
pub(crate) type EvalG =
    unsafe extern "C" fn(i32, *mut f64, bool, i32, *mut f64, *mut c_void) -> bool;
pub(crate) type EvalGradF =
    unsafe extern "C" fn(i32, *mut f64, bool, *mut f64, *mut c_void) -> bool;
pub(crate) type EvalJacG = unsafe extern "C" fn(
    i32,
    *mut f64,
    bool,
    i32,
    i32,
    *mut i32,
    *mut i32,
    *mut f64,
    *mut c_void,
) -> bool;
pub(crate) type EvalH = Option<
    unsafe extern "C" fn(
        i32,
        *mut f64,
        bool,
        f64,
        i32,
        *mut f64,
        bool,
        i32,
        *mut i32,
        *mut i32,
        *mut f64,
        *mut c_void,
    ) -> bool,
>;
pub(crate) type IntermediateCb = unsafe extern "C" fn(
    i32,
    i32,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    i32,
    *mut c_void,
) -> bool;
pub(crate) type SetIntermediateCallbackFn =
    unsafe extern "C" fn(IpoptProblem, IntermediateCb) -> bool;
pub(crate) type GetCurrentIterateFn = unsafe extern "C" fn(
    IpoptProblem,
    bool,
    i32,
    *mut f64,
    *mut f64,
    *mut f64,
    i32,
    *mut f64,
    *mut f64,
) -> bool;
pub(crate) type GetCurrentViolationsFn = unsafe extern "C" fn(
    IpoptProblem,
    bool,
    i32,
    *mut f64,
    *mut f64,
    *mut f64,
    *mut f64,
    *mut f64,
    i32,
    *mut f64,
    *mut f64,
) -> bool;

pub(crate) struct IpoptApi {
    _library: LibraryHandle,
    pub(crate) create_problem: unsafe extern "C" fn(
        i32,
        *mut f64,
        *mut f64,
        i32,
        *mut f64,
        *mut f64,
        i32,
        i32,
        i32,
        EvalF,
        EvalG,
        EvalGradF,
        EvalJacG,
        EvalH,
    ) -> IpoptProblem,
    pub(crate) free_problem: unsafe extern "C" fn(IpoptProblem),
    add_str_option: unsafe extern "C" fn(IpoptProblem, *mut c_char, *mut c_char) -> bool,
    add_num_option: unsafe extern "C" fn(IpoptProblem, *mut c_char, f64) -> bool,
    add_int_option: unsafe extern "C" fn(IpoptProblem, *mut c_char, i32) -> bool,
    pub(crate) solve: unsafe extern "C" fn(
        IpoptProblem,
        *mut f64,
        *mut f64,
        *mut f64,
        *mut f64,
        *mut f64,
        *mut f64,
        *mut c_void,
    ) -> i32,
    pub(crate) set_intermediate_callback: Option<SetIntermediateCallbackFn>,
    pub(crate) get_current_iterate: Option<GetCurrentIterateFn>,
    pub(crate) get_current_violations: Option<GetCurrentViolationsFn>,
}

impl IpoptApi {
    pub(crate) fn load(path: &Path) -> Result<Self, String> {
        let library = LibraryHandle::load(path)?;
        unsafe {
            Ok(Self {
                create_problem: library.symbol("CreateIpoptProblem")?,
                free_problem: library.symbol("FreeIpoptProblem")?,
                add_str_option: library.symbol("AddIpoptStrOption")?,
                add_num_option: library.symbol("AddIpoptNumOption")?,
                add_int_option: library.symbol("AddIpoptIntOption")?,
                solve: library.symbol("IpoptSolve")?,
                set_intermediate_callback: library.optional_symbol("SetIntermediateCallback")?,
                get_current_iterate: library.optional_symbol("GetIpoptCurrentIterate")?,
                get_current_violations: library.optional_symbol("GetIpoptCurrentViolations")?,
                _library: library,
            })
        }
    }

    pub(crate) fn add_str(
        &self,
        problem: IpoptProblem,
        key: &str,
        value: &str,
    ) -> Result<(), String> {
        let key = CString::new(key).map_err(|error| error.to_string())?;
        let value = CString::new(value).map_err(|error| error.to_string())?;
        let ok = unsafe {
            (self.add_str_option)(
                problem,
                key.as_ptr() as *mut c_char,
                value.as_ptr() as *mut c_char,
            )
        };
        if ok {
            Ok(())
        } else {
            Err(format!(
                "failed to set Ipopt string option {:?}={:?}",
                key.to_string_lossy(),
                value.to_string_lossy()
            ))
        }
    }

    pub(crate) fn add_num(
        &self,
        problem: IpoptProblem,
        key: &str,
        value: f64,
    ) -> Result<(), String> {
        let key = CString::new(key).map_err(|error| error.to_string())?;
        let ok = unsafe { (self.add_num_option)(problem, key.as_ptr() as *mut c_char, value) };
        if ok {
            Ok(())
        } else {
            Err(format!("failed to set Ipopt numeric option {key:?}"))
        }
    }

    pub(crate) fn add_int(
        &self,
        problem: IpoptProblem,
        key: &str,
        value: i32,
    ) -> Result<(), String> {
        let key = CString::new(key).map_err(|error| error.to_string())?;
        let ok = unsafe { (self.add_int_option)(problem, key.as_ptr() as *mut c_char, value) };
        if ok {
            Ok(())
        } else {
            Err(format!("failed to set Ipopt integer option {key:?}"))
        }
    }
}

pub(crate) struct IpoptProblemGuard {
    problem: IpoptProblem,
    free_problem: unsafe extern "C" fn(IpoptProblem),
}

impl IpoptProblemGuard {
    pub(crate) fn new(
        problem: IpoptProblem,
        free_problem: unsafe extern "C" fn(IpoptProblem),
    ) -> Self {
        Self {
            problem,
            free_problem,
        }
    }
}

impl Drop for IpoptProblemGuard {
    fn drop(&mut self) {
        unsafe {
            (self.free_problem)(self.problem);
        }
    }
}

#[cfg(feature = "ipopt-static-link")]
struct LibraryHandle;

#[cfg(feature = "ipopt-static-link")]
impl LibraryHandle {
    fn load(_path: &Path) -> Result<Self, String> {
        Ok(Self)
    }

    unsafe fn symbol<T: Copy>(&self, name: &str) -> Result<T, String> {
        let ptr = match name {
            "CreateIpoptProblem" => CreateIpoptProblem as *const c_void,
            "FreeIpoptProblem" => FreeIpoptProblem as *const c_void,
            "AddIpoptStrOption" => AddIpoptStrOption as *const c_void,
            "AddIpoptNumOption" => AddIpoptNumOption as *const c_void,
            "AddIpoptIntOption" => AddIpoptIntOption as *const c_void,
            "IpoptSolve" => IpoptSolve as *const c_void,
            "SetIntermediateCallback" => SetIntermediateCallback as *const c_void,
            "GetIpoptCurrentIterate" => GetIpoptCurrentIterate as *const c_void,
            "GetIpoptCurrentViolations" => GetIpoptCurrentViolations as *const c_void,
            _ => return Err(format!("missing static Ipopt symbol {name}")),
        };

        Ok(std::mem::transmute_copy(&ptr))
    }

    unsafe fn optional_symbol<T: Copy>(&self, name: &str) -> Result<Option<T>, String> {
        self.symbol(name).map(Some)
    }
}

#[cfg(feature = "ipopt-static-link")]
extern "C" {
    fn CreateIpoptProblem(
        n: i32,
        x_l: *mut f64,
        x_u: *mut f64,
        m: i32,
        g_l: *mut f64,
        g_u: *mut f64,
        nele_jac: i32,
        nele_hess: i32,
        index_style: i32,
        eval_f: EvalF,
        eval_g: EvalG,
        eval_grad_f: EvalGradF,
        eval_jac_g: EvalJacG,
        eval_h: EvalH,
    ) -> IpoptProblem;
    fn FreeIpoptProblem(problem: IpoptProblem);
    fn AddIpoptStrOption(problem: IpoptProblem, key: *mut c_char, value: *mut c_char) -> bool;
    fn AddIpoptNumOption(problem: IpoptProblem, key: *mut c_char, value: f64) -> bool;
    fn AddIpoptIntOption(problem: IpoptProblem, key: *mut c_char, value: i32) -> bool;
    fn SetIntermediateCallback(problem: IpoptProblem, intermediate_cb: IntermediateCb) -> bool;
    fn GetIpoptCurrentIterate(
        problem: IpoptProblem,
        scaled: bool,
        n: i32,
        x: *mut f64,
        z_l: *mut f64,
        z_u: *mut f64,
        m: i32,
        g: *mut f64,
        lambda: *mut f64,
    ) -> bool;
    fn GetIpoptCurrentViolations(
        problem: IpoptProblem,
        scaled: bool,
        n: i32,
        x_l_violation: *mut f64,
        x_u_violation: *mut f64,
        compl_x_l: *mut f64,
        compl_x_u: *mut f64,
        grad_lag_x: *mut f64,
        m: i32,
        nlp_constraint_violation: *mut f64,
        compl_g: *mut f64,
    ) -> bool;
    fn IpoptSolve(
        problem: IpoptProblem,
        x: *mut f64,
        g: *mut f64,
        obj_val: *mut f64,
        mult_g: *mut f64,
        mult_x_l: *mut f64,
        mult_x_u: *mut f64,
        user_data: *mut c_void,
    ) -> i32;
}

#[cfg(all(windows, not(feature = "ipopt-static-link")))]
struct LibraryHandle(*mut c_void);

#[cfg(all(windows, not(feature = "ipopt-static-link")))]
impl LibraryHandle {
    fn load(path: &Path) -> Result<Self, String> {
        use std::os::windows::ffi::OsStrExt;
        if let Some(parent) = path.parent() {
            let parent_wide = parent
                .as_os_str()
                .encode_wide()
                .chain(std::iter::once(0))
                .collect::<Vec<_>>();
            unsafe {
                let _ = SetDllDirectoryW(parent_wide.as_ptr());
            }
        }
        let wide = path
            .as_os_str()
            .encode_wide()
            .chain(std::iter::once(0))
            .collect::<Vec<_>>();
        let handle = unsafe { LoadLibraryW(wide.as_ptr()) };
        if handle.is_null() {
            Err(format!("failed to load Ipopt DLL: {}", path.display()))
        } else {
            Ok(Self(handle))
        }
    }

    unsafe fn symbol<T: Copy>(&self, name: &str) -> Result<T, String> {
        let name = CString::new(name).map_err(|error| error.to_string())?;
        let ptr = GetProcAddress(self.0, name.as_ptr());
        if ptr.is_null() {
            return Err(format!("missing Ipopt symbol {}", name.to_string_lossy()));
        }
        Ok(std::mem::transmute_copy(&ptr))
    }

    unsafe fn optional_symbol<T: Copy>(&self, name: &str) -> Result<Option<T>, String> {
        let name = CString::new(name).map_err(|error| error.to_string())?;
        let ptr = GetProcAddress(self.0, name.as_ptr());
        if ptr.is_null() {
            Ok(None)
        } else {
            Ok(Some(std::mem::transmute_copy(&ptr)))
        }
    }
}

#[cfg(all(windows, not(feature = "ipopt-static-link")))]
impl Drop for LibraryHandle {
    fn drop(&mut self) {
        unsafe {
            let _ = FreeLibrary(self.0);
        }
    }
}

#[cfg(all(windows, not(feature = "ipopt-static-link")))]
#[link(name = "kernel32")]
extern "system" {
    fn LoadLibraryW(lp_lib_file_name: *const u16) -> *mut c_void;
    fn SetDllDirectoryW(lp_path_name: *const u16) -> c_int;
    fn GetProcAddress(h_module: *mut c_void, lp_proc_name: *const c_char) -> *mut c_void;
    fn FreeLibrary(h_lib_module: *mut c_void) -> c_int;
}

#[cfg(all(not(windows), not(feature = "ipopt-static-link")))]
struct LibraryHandle(*mut c_void);

#[cfg(all(not(windows), not(feature = "ipopt-static-link")))]
impl LibraryHandle {
    fn load(path: &Path) -> Result<Self, String> {
        use std::os::unix::ffi::OsStrExt;

        let path = CString::new(path.as_os_str().as_bytes()).map_err(|error| error.to_string())?;
        let handle = unsafe { dlopen(path.as_ptr(), RTLD_NOW | RTLD_GLOBAL) };
        if handle.is_null() {
            Err(format!("failed to load Ipopt library: {}", dl_error()))
        } else {
            Ok(Self(handle))
        }
    }

    unsafe fn symbol<T: Copy>(&self, name: &str) -> Result<T, String> {
        let name = CString::new(name).map_err(|error| error.to_string())?;
        let ptr = dlsym(self.0, name.as_ptr());
        if ptr.is_null() {
            return Err(format!("missing Ipopt symbol {}", name.to_string_lossy()));
        }
        Ok(std::mem::transmute_copy(&ptr))
    }

    unsafe fn optional_symbol<T: Copy>(&self, name: &str) -> Result<Option<T>, String> {
        let name = CString::new(name).map_err(|error| error.to_string())?;
        let ptr = dlsym(self.0, name.as_ptr());
        if ptr.is_null() {
            Ok(None)
        } else {
            Ok(Some(std::mem::transmute_copy(&ptr)))
        }
    }
}

#[cfg(all(not(windows), not(feature = "ipopt-static-link")))]
impl Drop for LibraryHandle {
    fn drop(&mut self) {
        unsafe {
            let _ = dlclose(self.0);
        }
    }
}

#[cfg(all(
    not(windows),
    not(any(target_os = "macos", target_os = "ios")),
    not(feature = "ipopt-static-link")
))]
#[link(name = "dl")]
extern "C" {
    fn dlopen(filename: *const c_char, flag: c_int) -> *mut c_void;
    fn dlsym(handle: *mut c_void, symbol: *const c_char) -> *mut c_void;
    fn dlclose(handle: *mut c_void) -> c_int;
    fn dlerror() -> *const c_char;
}

#[cfg(all(
    any(target_os = "macos", target_os = "ios"),
    not(feature = "ipopt-static-link")
))]
extern "C" {
    fn dlopen(filename: *const c_char, flag: c_int) -> *mut c_void;
    fn dlsym(handle: *mut c_void, symbol: *const c_char) -> *mut c_void;
    fn dlclose(handle: *mut c_void) -> c_int;
    fn dlerror() -> *const c_char;
}

#[cfg(all(
    not(windows),
    not(any(target_os = "macos", target_os = "ios")),
    not(feature = "ipopt-static-link")
))]
const RTLD_GLOBAL: c_int = 0x100;

#[cfg(all(
    any(target_os = "macos", target_os = "ios"),
    not(feature = "ipopt-static-link")
))]
const RTLD_GLOBAL: c_int = 0x8;

#[cfg(all(not(windows), not(feature = "ipopt-static-link")))]
const RTLD_NOW: c_int = 0x2;

#[cfg(all(not(windows), not(feature = "ipopt-static-link")))]
fn dl_error() -> String {
    let error = unsafe { dlerror() };
    if error.is_null() {
        "unknown dlopen error".to_owned()
    } else {
        unsafe { std::ffi::CStr::from_ptr(error) }
            .to_string_lossy()
            .into_owned()
    }
}

pub(crate) fn status_name(code: i32) -> &'static str {
    match code {
        0 => "Solve_Succeeded",
        1 => "Solved_To_Acceptable_Level",
        2 => "Infeasible_Problem_Detected",
        3 => "Search_Direction_Becomes_Too_Small",
        4 => "Diverging_Iterates",
        5 => "User_Requested_Stop",
        6 => "Feasible_Point_Found",
        -1 => "Maximum_Iterations_Exceeded",
        -2 => "Restoration_Failed",
        -3 => "Error_In_Step_Computation",
        -4 => "Maximum_CpuTime_Exceeded",
        -5 => "Maximum_WallTime_Exceeded",
        -10 => "Not_Enough_Degrees_Of_Freedom",
        -11 => "Invalid_Problem_Definition",
        -12 => "Invalid_Option",
        -13 => "Invalid_Number_Detected",
        -100 => "Unrecoverable_Exception",
        -101 => "NonIpopt_Exception_Thrown",
        -102 => "Insufficient_Memory",
        -199 => "Internal_Error",
        _ => "Unknown_Ipopt_Status",
    }
}

pub(crate) fn status_is_success(code: i32) -> bool {
    matches!(code, 0 | 1)
}

#[cfg(test)]
mod tests {
    use super::status_is_success;

    #[test]
    fn ipopt_success_policy_rejects_early_failure_statuses() {
        assert!(status_is_success(0));
        assert!(status_is_success(1));
        assert!(!status_is_success(-1));
        assert!(!status_is_success(-11));
        assert!(!status_is_success(5));
    }
}
