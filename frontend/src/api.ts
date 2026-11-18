export async function request<T>(path: string, payload?: Record<string, unknown>): Promise<T> {
  const response = await fetch(path, {
    method: payload ? 'POST' : 'GET',
    signal: AbortSignal.timeout(5000),
    ...(payload
      ? {
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ ...payload, request_id: crypto.randomUUID() }),
        }
      : {}),
  })
  if (!response.ok) {
    let message = `请求失败（${response.status}）`
    try {
      const { detail } = await response.json()
      if (typeof detail === 'string') message = detail
      else if (typeof detail?.message === 'string') message = detail.message
      else if (Array.isArray(detail)) message = detail.map((item) => item.msg).join('；')
    } catch {
      // Keep the HTTP status for non-JSON errors.
    }
    throw new Error(message)
  }
  return (await response.json()) as T
}
