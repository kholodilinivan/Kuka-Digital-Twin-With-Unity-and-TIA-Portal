public class PLCGenericSend<T> : PLCSendItem
{
    public virtual void ChangeState(T value)
    {
        base.ChangeState(value);
    }
}
