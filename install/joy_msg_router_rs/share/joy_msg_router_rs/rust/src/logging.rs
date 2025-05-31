use safe_drive::logger::Logger;
use safe_drive::{pr_debug, pr_error, pr_info, pr_warn};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
}

pub struct LogContext<'a> {
    pub module: &'a str,
    pub function: &'a str,
    pub details: Option<&'a str>,
}

pub fn log_with_context(logger: &Logger, level: LogLevel, context: LogContext, message: &str) {
    let prefix = format!("[{}::{}]", context.module, context.function);
    let full_message = if let Some(details) = context.details {
        format!("{} {} - {}", prefix, message, details)
    } else {
        format!("{} {}", prefix, message)
    };

    match level {
        LogLevel::Debug => pr_debug!(logger, "{}", full_message),
        LogLevel::Info => pr_info!(logger, "{}", full_message),
        LogLevel::Warn => pr_warn!(logger, "{}", full_message),
        LogLevel::Error => pr_error!(logger, "{}", full_message),
    }
}

pub fn log_error(logger: &Logger, context: LogContext, error: &anyhow::Error) {
    log_with_context(logger, LogLevel::Error, context, &error.to_string());
}

/// Convenience macros for structured logging
#[macro_export]
macro_rules! log_debug {
    ($logger:expr, $module:expr, $function:expr, $msg:expr) => {
        $crate::logging::log_with_context(
            $logger,
            $crate::logging::LogLevel::Debug,
            $crate::logging::LogContext {
                module: $module,
                function: $function,
                details: None,
            },
            $msg,
        )
    };
    ($logger:expr, $module:expr, $function:expr, $msg:expr, $details:expr) => {
        $crate::logging::log_with_context(
            $logger,
            $crate::logging::LogLevel::Debug,
            $crate::logging::LogContext {
                module: $module,
                function: $function,
                details: Some($details),
            },
            $msg,
        )
    };
}

#[macro_export]
macro_rules! log_info {
    ($logger:expr, $module:expr, $function:expr, $msg:expr) => {
        $crate::logging::log_with_context(
            $logger,
            $crate::logging::LogLevel::Info,
            $crate::logging::LogContext {
                module: $module,
                function: $function,
                details: None,
            },
            $msg,
        )
    };
    ($logger:expr, $module:expr, $function:expr, $msg:expr, $details:expr) => {
        $crate::logging::log_with_context(
            $logger,
            $crate::logging::LogLevel::Info,
            $crate::logging::LogContext {
                module: $module,
                function: $function,
                details: Some($details),
            },
            $msg,
        )
    };
}

#[macro_export]
macro_rules! log_warn {
    ($logger:expr, $module:expr, $function:expr, $msg:expr) => {
        $crate::logging::log_with_context(
            $logger,
            $crate::logging::LogLevel::Warn,
            $crate::logging::LogContext {
                module: $module,
                function: $function,
                details: None,
            },
            $msg,
        )
    };
    ($logger:expr, $module:expr, $function:expr, $msg:expr, $details:expr) => {
        $crate::logging::log_with_context(
            $logger,
            $crate::logging::LogLevel::Warn,
            $crate::logging::LogContext {
                module: $module,
                function: $function,
                details: Some($details),
            },
            $msg,
        )
    };
}

#[macro_export]
macro_rules! log_error_detail {
    ($logger:expr, $module:expr, $function:expr, $error:expr) => {
        $crate::logging::log_error(
            $logger,
            $crate::logging::LogContext {
                module: $module,
                function: $function,
                details: None,
            },
            $error,
        )
    };
}

#[allow(dead_code)]
pub fn log_command_result(logger: &Logger, command: &str, result: anyhow::Result<()>) {
    match result {
        Ok(()) => log_with_context(
            logger,
            LogLevel::Debug,
            LogContext {
                module: "command_processor",
                function: "process",
                details: Some(command),
            },
            "Command processed successfully",
        ),
        Err(ref e) => log_error(
            logger,
            LogContext {
                module: "command_processor",
                function: "process",
                details: Some(command),
            },
            e,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_log_levels() {
        assert!(LogLevel::Debug != LogLevel::Error);
        assert_eq!(LogLevel::Info, LogLevel::Info);
    }

    #[test]
    fn test_log_context_formatting() {
        let context = LogContext {
            module: "test_module",
            function: "test_function",
            details: Some("extra info"),
        };

        // Test that context is properly structured
        assert_eq!(context.module, "test_module");
        assert_eq!(context.function, "test_function");
        assert_eq!(context.details, Some("extra info"));
    }
}
