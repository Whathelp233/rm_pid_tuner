#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM Interface for PID Auto-Tuning

This script provides a ROS service interface to call LLM APIs for PID parameter analysis.

Usage:
    # As a ROS service node
    rosrun rm_pid_tuner llm_interface.py

    # Call the service
    rosservice call /pid_tuner/llm_analyze "{...}"
"""

import rospy
import json
import re
import time
import os
from typing import Dict, Any, Optional

# 尝试导入 requests，如果失败则提供后备方案
try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False
    rospy.logwarn("[LLMInterface] requests module not found, will use fallback only")

from rm_pid_tuner.srv import LLMAnalyze, LLMAnalyzeResponse


class LLMInterface:
    """LLM API 客户端，用于 PID 参数分析和优化建议"""

    # 支持的 LLM 提供商配置
    PROVIDERS = {
        'deepseek': {
            'api_url': 'https://api.deepseek.com/v1',
            'model': 'deepseek-chat',
            'env_key': 'DEEPSEEK_API_KEY'
        },
        'minimax': {
            'api_url': 'https://api.minimax.chat/v1',
            'model': 'MiniMax-M2.5',
            'env_key': 'MINIMAX_API_KEY'
        },
        'openai': {
            'api_url': 'https://api.openai.com/v1',
            'model': 'gpt-4o',
            'env_key': 'OPENAI_API_KEY'
        }
    }

    def __init__(self):
        """初始化 LLM 接口，从 ROS 参数服务器或环境变量读取配置"""
        # 获取提供商类型
        self.provider = rospy.get_param('~llm/provider', 'deepseek')
        provider_config = self.PROVIDERS.get(self.provider, self.PROVIDERS['deepseek'])

        # 优先从环境变量读取 API Key，然后从 ROS 参数
        env_key_name = provider_config.get('env_key', 'DEEPSEEK_API_KEY')
        self.api_key = os.environ.get(env_key_name, '')
        if not self.api_key:
            self.api_key = os.environ.get('LLM_API_KEY', '')  # 通用环境变量
        if not self.api_key:
            self.api_key = rospy.get_param('~llm/api_key', '')

        # 其他配置从参数服务器读取，使用提供商默认值
        self.api_url = rospy.get_param('~llm/api_url', provider_config['api_url'])
        self.model = rospy.get_param('~llm/model', provider_config['model'])
        self.temperature = rospy.get_param('~llm/temperature', 0.3)
        self.max_tokens = rospy.get_param('~llm/max_tokens', 500)
        self.timeout = rospy.get_param('~llm/timeout', 30.0)

        # 安全检查
        if not self.api_key or self.api_key == 'your-api-key-here':
            rospy.logwarn("[LLMInterface] API Key not configured! "
                         "Will use fallback logic. "
                         f"Set {env_key_name} environment variable or ~llm/api_key parameter.")
            self.api_key = ''
        else:
            # 隐藏 API Key 的大部分字符
            masked_key = self.api_key[:4] + '****' + self.api_key[-4:] if len(self.api_key) > 8 else '****'
            rospy.loginfo(f"[LLMInterface] Initialized with provider: {self.provider}, model: {self.model}, API key: {masked_key}")

        # 系统 Prompt
        self.system_prompt = """你是一个 PID 控制算法专家。请分析以下控制系统数据，判断当前 PID 参数表现并给出优化建议。

## 可用数据
你将收到完整的控制系统数据，包括：
1. **系统信息**: 控制器类型、关节名称、当前 PID 参数、目标值、调参轮次
2. **性能指标**: MAE、RMSE、最大/最小误差、标准差、ITAE、ISE、IAE
3. **趋势分析**: 误差变化趋势、震荡过零次数、震荡程度
4. **完整时间序列**: 所有采样点的详细数据（时间戳、设定值、实际值、误差、控制输出、P/I/D项）

## 重要约束
- **禁止超调**：严禁让系统超过目标值，一旦发现超调必须立即减小 Kp 和增大 Kd
- **稳态优先**：优先消除稳态误差，再考虑响应速度
- **利用完整数据**：仔细分析时间序列数据中的趋势和模式，不要只看汇总指标
- **请直接给出参数**：不要建议运行其他脚本，直接根据你的经验给出新的 PID 参数
- **输出格式**：只输出纯 JSON，不要有任何 Markdown 标记或解释文字

## 分析建议
1. 检查时间序列数据中的误差趋势：是收敛、发散还是震荡？
2. 分析控制输出是否饱和：如果控制输出接近极限，可能需要调整参数
3. 观察 P/I/D 各项的贡献：哪一项对控制输出影响最大？
4. 根据震荡过零次数判断稳定性：过零多说明震荡严重

## 判断规则
- 震荡剧烈（过零次数>3）→ 减小 Kp 或增大 Kd
- 响应太慢（误差趋势稳定但MAE大）→ 增大 Kp（可以大胆增加，如 +50%）
- 稳态误差（趋势收敛但有残留误差）→ 增大 Ki（可以大胆增加，如 +50%）
- 超调过大 → 大幅减小 Kp（至少减少 30%）和增大 Kd（至少增加 50%）
- 误差发散 → 立即大幅减小所有增益

## 输出格式
请直接返回 JSON 格式:
{"analysis": "基于时间序列数据的简短分析", "p": 数值, "i": 数值, "d": 数值, "status": "TUNING 或 DONE"}"""

    def analyze(self, data_text: str, current_params: Dict[str, float],
                conservative_mode: bool = False) -> Dict[str, Any]:
        """
        调用 LLM 分析 PID 数据并返回优化建议

        Args:
            data_text: 格式化后的数据文本
            current_params: 当前 PID 参数字典 {'p': float, 'i': float, 'd': float}
            conservative_mode: 是否使用保守模式

        Returns:
            包含新 PID 参数的字典
        """
        # 检查是否有 API Key 和 requests 模块
        if not self.api_key or not HAS_REQUESTS:
            rospy.logdebug("[LLMInterface] No API key or requests module, using fallback logic")
            return self._fallback_analysis(data_text, current_params, conservative_mode)

        # 构建用户提示
        mode_hint = "【保守模式】" if conservative_mode else ""
        user_prompt = f"""{mode_hint}请分析以下 PID 控制系统数据并给出优化建议：

## 数据
{data_text}

请直接返回 JSON 格式的分析结果。"""

        # 重试机制：最多 3 次重试，指数退避
        max_retries = 3
        for attempt in range(max_retries):
            try:
                if attempt > 0:
                    wait_time = 2 ** attempt  # 指数退避：2, 4 秒
                    rospy.logwarn(f"[LLMInterface] Retry attempt {attempt + 1}/{max_retries}, waiting {wait_time}s...")
                    time.sleep(wait_time)

                rospy.loginfo(f"[LLMInterface] Calling LLM API (attempt {attempt + 1}/{max_retries})...")

                headers = {
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json"
                }

                payload = {
                    "model": self.model,
                    "messages": [
                        {"role": "system", "content": self.system_prompt},
                        {"role": "user", "content": user_prompt}
                    ],
                    "temperature": self.temperature,
                    "max_tokens": self.max_tokens
                }

                response = requests.post(
                    f"{self.api_url}/chat/completions",
                    headers=headers,
                    json=payload,
                    timeout=self.timeout
                )

                if response.status_code != 200:
                    # 判断是否为可重试的错误
                    if response.status_code in [429, 500, 502, 503, 504]:
                        rospy.logwarn(f"[LLMInterface] API call failed with retryable error: {response.status_code}")
                        if attempt < max_retries - 1:
                            continue  # 重试
                    rospy.logerr(f"[LLMInterface] API call failed: {response.status_code} - {response.text[:200]}")
                    return self._fallback_analysis(data_text, current_params, conservative_mode)

                resp_data = response.json()

                # 处理 MiniMax M2.5 的响应格式
                message = resp_data.get("choices", [{}])[0].get("message", {})
                result_text = message.get("content", "")
                if not result_text or result_text.strip() == "":
                    result_text = message.get("reasoning_content", "")

                rospy.logdebug(f"[LLMInterface] Raw response: {result_text[:200]}...")

                # 解析 JSON 响应
                parsed = self._parse_json_response(result_text, current_params)
                if parsed:
                    parsed['used_fallback'] = False
                    if attempt > 0:
                        parsed['analysis'] = f"[重试成功] {parsed.get('analysis', '')}"
                    return parsed
                else:
                    return self._fallback_analysis(data_text, current_params, conservative_mode)

            except requests.exceptions.Timeout:
                rospy.logwarn(f"[LLMInterface] API call timeout (attempt {attempt + 1}/{max_retries})")
                if attempt < max_retries - 1:
                    continue  # 重试
                return self._fallback_analysis(data_text, current_params, conservative_mode)

            except requests.exceptions.ConnectionError as e:
                rospy.logwarn(f"[LLMInterface] Connection error (attempt {attempt + 1}/{max_retries}): {e}")
                if attempt < max_retries - 1:
                    continue  # 重试
                return self._fallback_analysis(data_text, current_params, conservative_mode)

            except requests.exceptions.RequestException as e:
                rospy.logerr(f"[LLMInterface] Request error: {e}")
                return self._fallback_analysis(data_text, current_params, conservative_mode)

            except Exception as e:
                rospy.logerr(f"[LLMInterface] Unexpected error: {e}")
                return self._fallback_analysis(data_text, current_params, conservative_mode)

        # 所有重试都失败
        return self._fallback_analysis(data_text, current_params, conservative_mode)

    def _parse_json_response(self, text: str, current_params: Dict[str, float]) -> Optional[Dict[str, Any]]:
        """解析 LLM 返回的 JSON 响应"""
        # 预处理：去掉 Markdown 代码块标记
        text = text.replace("```json", "").replace("```", "").strip()

        # 方法1: 尝试直接解析
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass

        # 方法2: 提取 JSON 块
        start = text.find('{')
        end = text.rfind('}')
        if start != -1 and end != -1 and end > start:
            json_str = text[start:end+1]
            try:
                parsed = json.loads(json_str)
                if 'p' in parsed or 'i' in parsed:
                    return parsed
            except json.JSONDecodeError as e:
                rospy.logwarn(f"[LLMInterface] JSON block parse failed: {e}")

        # 方法3: 正则提取 key-value
        p_match = re.search(r'"p"\s*:\s*([0-9.+-]+)', text)
        i_match = re.search(r'"i"\s*:\s*([0-9.+-]+)', text)
        d_match = re.search(r'"d"\s*:\s*([0-9.+-]+)', text)

        if p_match and i_match and d_match:
            analysis_match = re.search(r'"analysis"\s*:\s*"([^"]*)"', text)
            status_match = re.search(r'"status"\s*:\s*"([^"]*)"', text)

            return {
                "analysis": analysis_match.group(1) if analysis_match else "解析成功",
                "p": float(p_match.group(1)),
                "i": float(i_match.group(1)),
                "d": float(d_match.group(1)),
                "status": status_match.group(1) if status_match else "TUNING"
            }

        rospy.logerr("[LLMInterface] Failed to parse response")
        return None

    def _fallback_analysis(self, data_text: str, current_params: Dict[str, float],
                           conservative_mode: bool = False) -> Dict[str, Any]:
        """
        后备分析逻辑：当 LLM API 不可用时使用简单的规则进行调整
        """
        # 从数据文本中提取误差信息
        avg_error = 0.0
        error_stddev = 0.0

        # 解析数据
        lines = data_text.strip().split('\n')
        errors = []
        for line in lines:
            if ',' in line:
                parts = line.split(',')
                try:
                    # 假设最后一个值是误差
                    error = float(parts[-1].strip())
                    errors.append(abs(error))
                except (ValueError, IndexError):
                    continue

        if errors:
            avg_error = sum(errors) / len(errors)
            if len(errors) > 1:
                variance = sum((e - avg_error) ** 2 for e in errors) / len(errors)
                error_stddev = variance ** 0.5

        # 尝试从文本中提取预计算的指标
        for line in lines:
            if '平均误差' in line or 'avg_error' in line.lower():
                try:
                    match = re.search(r'[:：]\s*([0-9.]+)', line)
                    if match:
                        avg_error = float(match.group(1))
                except (ValueError, AttributeError):
                    pass
            if '误差标准差' in line or 'stddev' in line.lower():
                try:
                    match = re.search(r'[:：]\s*([0-9.]+)', line)
                    if match:
                        error_stddev = float(match.group(1))
                except (ValueError, AttributeError):
                    pass

        # 根据误差调整参数
        p = current_params.get('p', 1.0)
        i = current_params.get('i', 0.1)
        d = current_params.get('d', 0.05)

        # 保守模式：降低增益
        if conservative_mode:
            p *= 0.6
            i *= 0.6
            analysis = "[后备模式-保守] 降低增益以提高稳定性"
            return {
                "analysis": analysis,
                "p": p,
                "i": i,
                "d": d,
                "status": "TUNING",
                "used_fallback": True
            }

        # 根据误差特性调整
        if error_stddev > 1.5:
            # 震荡，减小 Kp，增大 Kd
            p *= 0.85
            d *= 1.3
            analysis = "[后备模式] 检测到震荡，减小Kp，增大Kd"
        elif avg_error > 10:
            # 响应太慢，增大 Kp
            p *= 1.3
            analysis = "[后备模式] 响应较慢，增大Kp"
        elif avg_error > 2:
            # 稳态误差，增大 Ki
            i *= 1.4
            analysis = "[后备模式] 存在稳态误差，增大Ki"
        elif avg_error > 0.5:
            # 微调
            p *= 1.1
            i *= 1.1
            analysis = "[后备模式] 微调参数"
        else:
            analysis = "[后备模式] 性能良好，保持当前参数"

        return {
            "analysis": analysis,
            "p": p,
            "i": i,
            "d": d,
            "status": "TUNING" if avg_error > 0.3 else "DONE",
            "used_fallback": True
        }


class LLMServiceNode:
    """ROS 服务节点，提供 LLM 分析服务"""

    def __init__(self):
        rospy.init_node('llm_interface', anonymous=True)

        self.llm = LLMInterface()

        # 创建服务
        self.service = rospy.Service('~llm_analyze', LLMAnalyze, self.handle_analyze)

        rospy.loginfo(f"[LLMServiceNode] LLM Interface service ready at {rospy.get_name()}/llm_analyze")

    def handle_analyze(self, req):
        """处理 LLM 分析请求"""
        start_time = time.time()

        rospy.loginfo(f"[LLMServiceNode] Analyzing request for {req.controller_name}/{req.joint_name}, round {req.round_number}")

        # 构建当前参数
        current_params = {
            'p': req.current_p,
            'i': req.current_i,
            'd': req.current_d
        }

        try:
            # 调用 LLM 分析
            result = self.llm.analyze(
                req.data_text,
                current_params,
                req.conservative_mode
            )

            response_time = time.time() - start_time

            # 构建响应
            response = LLMAnalyzeResponse()
            response.success = True
            response.message = "Analysis completed"
            response.analysis = result.get('analysis', '')
            response.suggested_p = result.get('p', req.current_p)
            response.suggested_i = result.get('i', req.current_i)
            response.suggested_d = result.get('d', req.current_d)
            response.status = result.get('status', 'TUNING')
            response.used_fallback = result.get('used_fallback', True)
            response.response_time = response_time

            rospy.loginfo(f"[LLMServiceNode] Analysis done in {response_time:.2f}s, "
                         f"fallback={response.used_fallback}, "
                         f"P={response.suggested_p:.4f}, I={response.suggested_i:.4f}, D={response.suggested_d:.4f}")

            return response

        except Exception as e:
            rospy.logerr(f"[LLMServiceNode] Error during analysis: {e}")
            response = LLMAnalyzeResponse()
            response.success = False
            response.message = str(e)
            response.analysis = f"Error: {e}"
            response.suggested_p = req.current_p
            response.suggested_i = req.current_i
            response.suggested_d = req.current_d
            response.status = "ERROR"
            response.used_fallback = True
            response.response_time = time.time() - start_time
            return response

    def spin(self):
        """保持节点运行"""
        rospy.spin()


if __name__ == "__main__":
    try:
        node = LLMServiceNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
