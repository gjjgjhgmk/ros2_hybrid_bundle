/**
 * 公共JavaScript工具库
 * 提供通用的DOM操作、API调用、消息显示等功能
 */

// ==================== 常量定义 ====================
const API_BASE = window.location.origin;

// 输入框宽度常量
const INPUT_MIN_WIDTH = 150;  // 最小宽度（像素）
const INPUT_MAX_WIDTH = 1200; // 最大宽度（像素）

// ==================== DOM工具函数 ====================

/**
 * 创建DOM元素
 * @param {string} tag - 标签名
 * @param {string} className - CSS类名
 * @param {string} textContent - 文本内容
 * @returns {HTMLElement} 创建的元素
 */
function createElement(tag, className = '', textContent = '') {
    const el = document.createElement(tag);
    if (className) el.className = className;
    if (textContent) el.textContent = textContent;
    return el;
}

/**
 * 创建按钮元素
 * @param {string} text - 按钮文本
 * @param {string} className - CSS类名
 * @param {Function} onClick - 点击事件处理函数
 * @returns {HTMLButtonElement} 创建的按钮
 */
function createButton(text, className = 'btn btn-secondary', onClick = null) {
    const btn = createElement('button', className, text);
    if (onClick) btn.onclick = onClick;
    return btn;
}

// ==================== API调用工具 ====================

/**
 * 获取API完整URL
 * @param {string} endpoint - API端点
 * @returns {string} 完整的API URL
 */
function getApiUrl(endpoint) {
    return `${API_BASE}${endpoint}`;
}

/**
 * 统一的API调用函数
 * @param {string} endpoint - API端点
 * @param {Object} options - 请求选项
 * @returns {Promise<Object>} API响应数据
 */
async function apiCall(endpoint, options = {}) {
    try {
        const response = await fetch(getApiUrl(endpoint), {
            headers: { 'Content-Type': 'application/json', ...options.headers },
            ...options
        });

        // 检查响应状态
        if (!response.ok) {
            let errorMessage = `HTTP ${response.status}: ${response.statusText}`;
            try {
                const errorData = await response.json();
                if (errorData.error) {
                    errorMessage = errorData.error;
                }
            } catch (e) {
                try {
                    const text = await response.text();
                    if (text && text.length < 200) {
                        errorMessage = text;
                    }
                } catch (e2) {
                    // 忽略
                }
            }
            throw new Error(errorMessage);
        }

        // 检查Content-Type是否为JSON
        const contentType = response.headers.get('content-type');
        if (contentType && contentType.includes('application/json')) {
            return await response.json();
        } else {
            const text = await response.text();
            try {
                return JSON.parse(text);
            } catch (e) {
                throw new Error(`服务器返回了非JSON响应: ${text.substring(0, 100)}...`);
            }
        }
    } catch (error) {
        if (error.message) {
            throw error;
        }
        throw new Error(`API调用失败: ${error.message}`);
    }
}

/**
 * 通用API调用函数（带回调）
 * @param {string} endpoint - API端点
 * @param {Object} data - 请求数据
 * @param {Object} options - 选项配置
 * @returns {Promise<Object>} API响应数据
 */
async function callAPI(endpoint, data, options = {}) {
    const {
        onSuccess,
        onError,
        loadingText = '处理中...',
        successText = '操作成功',
        errorText = '操作失败'
    } = options;

    try {
        const response = await fetch(`${API_BASE}${endpoint}`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(data)
        });

        const result = await response.json();

        if (result.success) {
            if (onSuccess) onSuccess(result);
            else showMessage(successText, 'success');
            return result;
        } else {
            const errorMsg = result.error || errorText;
            if (onError) onError(result, errorMsg);
            else showMessage(errorMsg, 'error');
            throw new Error(errorMsg);
        }
    } catch (error) {
        const errorMsg = error.message || errorText;
        if (onError) onError(null, errorMsg);
        else showMessage(errorMsg, 'error');
        throw error;
    }
}

// ==================== 消息显示工具 ====================

/**
 * 显示消息
 * @param {string} message - 消息内容
 * @param {string} type - 消息类型 ('success', 'error', 'info')
 * @param {string|HTMLElement} containerId - 容器ID或元素
 * @param {number} duration - 显示时长（毫秒）
 */
function showMessage(message, type = 'error', containerId = null, duration = null) {
    const container = containerId
        ? (typeof containerId === 'string' ? document.getElementById(containerId) : containerId)
        : document.body;

    if (!container) {
        console.warn('消息容器不存在:', containerId);
        return;
    }

    const msgDiv = createElement('div', `message message-${type}`, message);

    // 如果容器有firstChild，插入到前面，否则追加
    if (container.firstChild) {
        container.insertBefore(msgDiv, container.firstChild);
    } else {
        container.appendChild(msgDiv);
    }

    const autoRemoveDuration = duration || (type === 'error' ? 5000 : 3000);
    setTimeout(() => msgDiv.remove(), autoRemoveDuration);
}

// ==================== 按钮状态管理 ====================

/**
 * 设置按钮加载状态
 * @param {string} buttonId - 按钮ID
 * @param {string} textId - 文本元素ID（可选）
 * @param {boolean} loading - 是否加载中
 * @param {string} text - 按钮文本
 */
function setButtonLoading(buttonId, textId, loading, text) {
    const button = document.getElementById(buttonId);
    const textSpan = textId ? document.getElementById(textId) : null;

    if (button) {
        button.disabled = loading;
        if (textSpan) {
            textSpan.innerHTML = loading
                ? `<span class="loading"></span>${text}`
                : text;
        } else if (!textId) {
            button.textContent = loading ? `处理中...` : text;
        }
    }
}

// ==================== 工具函数 ====================

/**
 * 复制文本到剪贴板
 * @param {string} text - 要复制的文本
 * @returns {Promise<boolean>} 是否成功
 */
async function copyToClipboard(text) {
    try {
        await navigator.clipboard.writeText(text);
        return true;
    } catch (err) {
        // 降级方案
        const textarea = document.createElement('textarea');
        textarea.value = text;
        textarea.style.position = 'fixed';
        textarea.style.opacity = '0';
        document.body.appendChild(textarea);
        textarea.select();
        try {
            document.execCommand('copy');
            document.body.removeChild(textarea);
            return true;
        } catch (e) {
            document.body.removeChild(textarea);
            return false;
        }
    }
}

/**
 * 防抖函数
 * @param {Function} func - 要防抖的函数
 * @param {number} wait - 等待时间（毫秒）
 * @returns {Function} 防抖后的函数
 */
function debounce(func, wait) {
    let timeout;
    return function executedFunction(...args) {
        const later = () => {
            clearTimeout(timeout);
            func(...args);
        };
        clearTimeout(timeout);
        timeout = setTimeout(later, wait);
    };
}

/**
 * 节流函数
 * @param {Function} func - 要节流的函数
 * @param {number} limit - 时间限制（毫秒）
 * @returns {Function} 节流后的函数
 */
function throttle(func, limit) {
    let inThrottle;
    return function (...args) {
        if (!inThrottle) {
            func.apply(this, args);
            inThrottle = true;
            setTimeout(() => inThrottle = false, limit);
        }
    };
}

/**
 * 格式化文件大小
 * @param {number} bytes - 字节数
 * @returns {string} 格式化后的字符串
 */
function formatFileSize(bytes) {
    if (bytes === 0) return '0 Bytes';
    const k = 1024;
    const sizes = ['Bytes', 'KB', 'MB', 'GB', 'TB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return Math.round(bytes / Math.pow(k, i) * 100) / 100 + ' ' + sizes[i];
}

/**
 * 格式化时间
 * @param {Date|number} date - 日期对象或时间戳
 * @returns {string} 格式化后的时间字符串
 */
function formatTime(date) {
    const d = date instanceof Date ? date : new Date(date);
    return d.toLocaleString('zh-CN', {
        year: 'numeric',
        month: '2-digit',
        day: '2-digit',
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
    });
}

// ==================== 输入框宽度调整 ====================

/**
 * 根据内容动态调整输入框宽度
 * @param {HTMLElement} input - 输入框元素
 * @param {number} minWidth - 最小宽度（像素），默认使用 INPUT_MIN_WIDTH
 * @param {number} maxWidth - 最大宽度（像素），默认使用 INPUT_MAX_WIDTH
 */
function adjustInputWidth(input, minWidth = INPUT_MIN_WIDTH, maxWidth = INPUT_MAX_WIDTH) {
    // checkbox 不需要调整宽度
    if (input.type === 'checkbox') {
        return;
    }

    // 获取父容器的可用宽度
    const getParentAvailableWidth = (element) => {
        let parent = element.parentElement;
        while (parent) {
            const parentStyle = window.getComputedStyle(parent);
            const parentWidth = parent.offsetWidth;
            const paddingLeft = parseFloat(parentStyle.paddingLeft) || 0;
            const paddingRight = parseFloat(parentStyle.paddingRight) || 0;
            const availableWidth = parentWidth - paddingLeft - paddingRight;

            // 如果父容器有明确的宽度限制，返回可用宽度
            if (parentWidth > 0 && availableWidth > 0) {
                return availableWidth;
            }
            parent = parent.parentElement;
        }
        return null;
    };

    const parentAvailableWidth = getParentAvailableWidth(input);
    const effectiveMaxWidth = parentAvailableWidth
        ? Math.min(maxWidth, parentAvailableWidth)
        : maxWidth;

    // textarea 需要特殊处理（多行文本）
    if (input.tagName === 'TEXTAREA') {
        // 对于 textarea，只调整宽度，高度由 rows 控制
        const temp = document.createElement('span');
        temp.style.visibility = 'hidden';
        temp.style.position = 'absolute';
        temp.style.whiteSpace = 'pre';
        temp.style.fontSize = window.getComputedStyle(input).fontSize;
        temp.style.fontFamily = window.getComputedStyle(input).fontFamily;
        temp.style.fontWeight = window.getComputedStyle(input).fontWeight;
        temp.style.letterSpacing = window.getComputedStyle(input).letterSpacing;

        // 对于多行文本，取最长的一行
        const lines = (input.value || input.placeholder || 'M').split('\n');
        const longestLine = lines.reduce((a, b) => a.length > b.length ? a : b, '');
        temp.textContent = longestLine || 'M';

        document.body.appendChild(temp);
        const textWidth = temp.offsetWidth;
        document.body.removeChild(temp);

        const computedStyle = window.getComputedStyle(input);
        const paddingLeft = parseFloat(computedStyle.paddingLeft) || 8;
        const paddingRight = parseFloat(computedStyle.paddingRight) || 8;
        const borderLeft = parseFloat(computedStyle.borderLeftWidth) || 1;
        const borderRight = parseFloat(computedStyle.borderRightWidth) || 1;

        const totalWidth = textWidth + paddingLeft + paddingRight + borderLeft + borderRight;
        const newWidth = totalWidth > effectiveMaxWidth
            ? effectiveMaxWidth
            : Math.max(minWidth, totalWidth);
        input.style.width = newWidth + 'px';
        return;
    }

    // 单行输入框（text, number等）
    const temp = document.createElement('span');
    temp.style.visibility = 'hidden';
    temp.style.position = 'absolute';
    temp.style.whiteSpace = 'pre';
    temp.style.fontSize = window.getComputedStyle(input).fontSize;
    temp.style.fontFamily = window.getComputedStyle(input).fontFamily;
    temp.style.fontWeight = window.getComputedStyle(input).fontWeight;
    temp.style.letterSpacing = window.getComputedStyle(input).letterSpacing;
    temp.textContent = input.value || input.placeholder || 'M';
    document.body.appendChild(temp);

    const textWidth = temp.offsetWidth;
    document.body.removeChild(temp);

    // 获取输入框的 padding 和 border
    const computedStyle = window.getComputedStyle(input);
    const paddingLeft = parseFloat(computedStyle.paddingLeft) || 8;
    const paddingRight = parseFloat(computedStyle.paddingRight) || 8;
    const borderLeft = parseFloat(computedStyle.borderLeftWidth) || 1;
    const borderRight = parseFloat(computedStyle.borderRightWidth) || 1;

    // 设置宽度，限制在最小值和最大值之间，且不超过父容器宽度
    const totalWidth = textWidth + paddingLeft + paddingRight + borderLeft + borderRight;
    // 如果内容超过最大宽度，使用最大宽度而不是压缩
    const newWidth = totalWidth > effectiveMaxWidth
        ? effectiveMaxWidth
        : Math.max(minWidth, totalWidth);
    input.style.width = newWidth + 'px';
}

/**
 * 为输入框添加自动宽度调整功能
 * @param {HTMLElement} input - 输入框元素
 * @param {number} minWidth - 最小宽度（像素）
 * @param {number} maxWidth - 最大宽度（像素）
 */
function enableAutoWidthAdjust(input, minWidth = INPUT_MIN_WIDTH, maxWidth = INPUT_MAX_WIDTH) {
    // 初始调整
    adjustInputWidth(input, minWidth, maxWidth);

    // 监听输入事件
    input.addEventListener('input', () => {
        adjustInputWidth(input, minWidth, maxWidth);
    });

    // 监听内容变化（包括粘贴等）
    input.addEventListener('change', () => {
        adjustInputWidth(input, minWidth, maxWidth);
    });
}

// ==================== 导出 ====================
if (typeof module !== 'undefined' && module.exports) {
    module.exports = {
        createElement,
        createButton,
        getApiUrl,
        apiCall,
        callAPI,
        showMessage,
        setButtonLoading,
        copyToClipboard,
        debounce,
        throttle,
        formatFileSize,
        formatTime,
        adjustInputWidth,
        enableAutoWidthAdjust,
        INPUT_MIN_WIDTH,
        INPUT_MAX_WIDTH
    };
}

